use crate::block_node::apply_state;
use crate::circuit::{Circuit, NodeId};
use crate::graph::{NodeKind, NodeState};
use crate::nodes::{Changes, Nodes, Snapshot};
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_redstone::sound::Sound;
use mchprs_simulation::block_changes::BlockChanges;
use mchprs_simulation::layout::{Component, Layout};
use mchprs_simulation::watch::{Bit, Breakpoint, Watch};
use mchprs_simulation::write_back::WriteBack;
use mchprs_simulation::{Blocks, End};
use rustc_hash::FxHashMap;

type NodeBlocks = Box<[(BlockPos, Block)]>;

/// Where the nodes of a circuit stand in a plot, and the blocks it last showed there.
#[derive(Debug)]
pub struct BlockMap {
    circuit: Circuit,
    region: Region,
    nodes_by_position: FxHashMap<BlockPos, NodeId>,
    blocks: Box<[NodeBlocks]>,
    states: Box<[NodeState]>,
}

impl BlockMap {
    pub fn new(circuit: Circuit, region: Region) -> BlockMap {
        let mut nodes_by_position = FxHashMap::default();
        for id in circuit.ids() {
            for &(pos, _) in &circuit.node(id).blocks {
                nodes_by_position.insert(pos, id);
            }
        }
        BlockMap {
            nodes_by_position,
            blocks: circuit
                .ids()
                .map(|id| {
                    let blocks = circuit.node(id).blocks.iter();
                    let blocks = blocks.map(|&(pos, id)| {
                        let block = Block::from_id(id);
                        (pos, block.expect("a circuit holds known block states"))
                    });
                    blocks.collect()
                })
                .collect(),
            states: circuit.ids().map(|id| circuit.node(id).state).collect(),
            circuit,
            region,
        }
    }

    pub fn layout(&self) -> Layout {
        let components = self.circuit.ids().flat_map(|id| {
            let node = self.circuit.node(id);
            let component = Component {
                port: node.port,
                observed: node.observed,
            };
            node.blocks.iter().map(move |&(pos, _)| (pos, component))
        });
        components.collect()
    }

    pub fn node_at(&self, pos: BlockPos) -> Option<NodeId> {
        self.nodes_by_position.get(&pos).copied()
    }

    /// The position of the node's first block. A node that takes input has one block.
    pub fn position(&self, node: NodeId) -> Option<BlockPos> {
        let blocks = self.blocks.get(node.index())?;
        blocks.first().map(|&(pos, _)| pos)
    }

    pub fn watch(&self, watch: Watch<BlockPos>) -> Watch<NodeId> {
        let node_bit = |bit| match bit {
            Bit::At(pos) => self.node_at(pos).map_or(Bit::Fixed(false), Bit::At),
            Bit::Fixed(on) => Bit::Fixed(on),
        };
        let breakpoints = watch.breakpoints.into_iter().map(|breakpoint| Breakpoint {
            id: breakpoint.id,
            bits: breakpoint.bits.iter().copied().map(node_bit).collect(),
            condition: breakpoint.condition,
        });
        Watch {
            breakpoints: breakpoints.collect(),
        }
    }

    /// A block that changed and is back as it was sounds once.
    pub fn record(&mut self, changes: &Changes, block_changes: &mut BlockChanges) {
        for &(node, state) in &changes.states {
            self.record_state(node, state, block_changes);
        }
        for &node in &changes.played_note_blocks {
            let NodeKind::NoteBlock { instrument, note } = self.circuit.node(node).kind else {
                continue;
            };
            let sound = Sound::note(instrument, note);
            let positions = self.blocks[node.index()].iter().map(|&(pos, _)| pos);
            block_changes
                .sounds
                .extend(positions.map(|pos| (pos, sound)));
        }
    }

    /// Compares blocks, not states: at compile time a block can disagree with its node.
    pub fn write_back(&self, snapshot: &Snapshot) -> WriteBack {
        let mut changes = BlockChanges::default();
        let shown = self.blocks.iter().zip(&self.states);
        for ((blocks, shown_state), &state) in shown.zip(&snapshot.states) {
            let reached = blocks.iter().map(|&(pos, mut block)| {
                apply_state(&mut block, state);
                (pos, block)
            });
            let reached: Vec<_> = reached.collect();
            let strength_changed = shown_state.power != state.power;
            let changed = reached.iter().zip(blocks).any(|(&(_, block), shown)| {
                (block != shown.1) || strength_changed && matches!(block, Block::Comparator(_))
            });
            if changed {
                push_blocks(&reached, state, &mut changes);
            }
        }
        let mut ticks: Vec<_> = snapshot
            .pending_ticks
            .iter()
            .flat_map(|&(node, tick)| {
                let blocks = self.blocks[node.index()].iter();
                blocks.map(move |&(pos, _)| (pos, tick))
            })
            .collect();
        ticks.sort_by_key(|&(_, tick)| tick);
        WriteBack {
            changes,
            ticks,
            region: self.region,
        }
    }

    pub fn end(&mut self, end: End<Nodes>) -> End<Blocks> {
        let mut changes = BlockChanges::default();
        self.record(&end.changes, &mut changes);
        End {
            changes,
            state: end.state.map(|snapshot| self.write_back(&snapshot)),
        }
    }

    fn record_state(&mut self, node: NodeId, state: NodeState, block_changes: &mut BlockChanges) {
        let index = node.index();
        let strength_changed = self.states[index].power != state.power;
        self.states[index] = state;
        let mut changed = false;
        for (pos, block) in &mut self.blocks[index] {
            let previous = *block;
            apply_state(block, state);
            let mut before = previous;
            if *block == previous {
                let reverted = NodeState {
                    power: (!state.is_powered()).into(),
                    ..state
                };
                apply_state(&mut before, reverted);
            }
            let sound = Sound::of_change(before, *block);
            block_changes
                .sounds
                .extend(sound.map(|sound| (*pos, sound)));
            changed |=
                *block != previous || strength_changed && matches!(block, Block::Comparator(_));
        }
        if changed {
            push_blocks(&self.blocks[index], state, block_changes);
        }
    }
}

fn push_blocks(blocks: &[(BlockPos, Block)], state: NodeState, block_changes: &mut BlockChanges) {
    for &(pos, block) in blocks {
        block_changes.blocks.push((pos, block));
        if matches!(block, Block::Comparator(_)) {
            let output_strength = state.power.into();
            let comparator = BlockEntity::Comparator { output_strength };
            block_changes.block_entities.push((pos, Some(comparator)));
        }
    }
}
