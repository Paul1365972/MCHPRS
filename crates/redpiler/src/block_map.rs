use crate::backend::Batch;
use crate::compile_graph::{NodeState, NodeType};
use crate::engine::{NodeChanges, PendingTick};
use crate::netlist::{Netlist, NodeId};
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::BlockPos;
use mchprs_world::TickEntry;
use rustc_hash::FxHashMap;
use smallvec::SmallVec;

type NodeBlocks = SmallVec<[(BlockPos, Block); 1]>;

pub struct BlockMap {
    blocks: Box<[NodeBlocks]>,
    types: Box<[NodeType]>,
    states: Box<[NodeState]>,
    nodes_by_position: FxHashMap<BlockPos, NodeId>,
}

impl BlockMap {
    pub fn new(netlist: &Netlist) -> BlockMap {
        let mut nodes_by_position = FxHashMap::default();
        let mut blocks = Vec::with_capacity(netlist.len());
        let mut types = Vec::with_capacity(netlist.len());
        let mut states = Vec::with_capacity(netlist.len());
        for id in netlist.ids() {
            let node = netlist.node(id);
            let node_blocks: NodeBlocks = node
                .block
                .iter()
                .map(|&(pos, block_id)| (pos, Block::from_id(block_id)))
                .collect();
            for &(pos, _) in &node_blocks {
                nodes_by_position.insert(pos, id);
            }
            blocks.push(node_blocks);
            types.push(node.ty.clone());
            states.push(node.state);
        }
        BlockMap {
            blocks: blocks.into_boxed_slice(),
            types: types.into_boxed_slice(),
            states: states.into_boxed_slice(),
            nodes_by_position,
        }
    }

    pub fn node_at(&self, pos: BlockPos) -> Option<NodeId> {
        self.nodes_by_position.get(&pos).copied()
    }

    pub fn node_type(&self, node: NodeId) -> &NodeType {
        &self.types[node.index()]
    }

    pub fn translate_ticks(&self, ticks: &[PendingTick]) -> Vec<TickEntry> {
        ticks
            .iter()
            .flat_map(|tick| {
                self.positions(tick.node).map(|pos| TickEntry {
                    pos,
                    ticks_left: tick.delay,
                    tick_priority: tick.priority,
                })
            })
            .collect()
    }

    pub fn map_ticks(&self, entries: &[TickEntry]) -> Vec<PendingTick> {
        entries
            .iter()
            .filter_map(|entry| {
                Some(PendingTick {
                    node: self.node_at(entry.pos)?,
                    delay: entry.ticks_left,
                    priority: entry.tick_priority,
                })
            })
            .collect()
    }

    pub fn translate(&mut self, changes: &NodeChanges) -> Batch {
        let mut out = Batch::default();
        for &(node, state) in &changes.states {
            self.record_state(node, state, &mut out);
        }
        for &node in &changes.notes {
            let NodeType::NoteBlock { instrument, note } = self.types[node.index()] else {
                panic!("node {node:?} is not a note block");
            };
            for pos in self.positions(node) {
                out.notes.push((pos, instrument, note));
            }
        }
        out
    }

    fn positions(&self, node: NodeId) -> impl Iterator<Item = BlockPos> + '_ {
        self.blocks[node.index()].iter().map(|&(pos, _)| pos)
    }

    fn record_state(&mut self, node: NodeId, state: NodeState, out: &mut Batch) {
        if self.states[node.index()] == state {
            return;
        }
        self.states[node.index()] = state;
        let is_comparator = matches!(self.types[node.index()], NodeType::Comparator { .. });
        for (pos, block) in &mut self.blocks[node.index()] {
            set_block_state(block, state);
            out.blocks.push((*pos, *block));
            if is_comparator {
                let block_entity = BlockEntity::Comparator {
                    output_strength: state.output_strength,
                };
                out.block_entities.push((*pos, block_entity));
            }
        }
    }
}

fn set_block_state(block: &mut Block, state: NodeState) {
    if let Some(powered) = block_powered_mut(block) {
        *powered = state.powered;
    }
    match block {
        Block::IronTrapdoor { open, .. } => *open = state.powered,
        Block::RedstoneWire(wire) => wire.power = state.output_strength,
        Block::Repeater(repeater) => repeater.locked = state.repeater_locked,
        _ => {}
    }
}

fn block_powered_mut(block: &mut Block) -> Option<&mut bool> {
    Some(match block {
        Block::Comparator(comparator) => &mut comparator.powered,
        Block::RedstoneTorch { lit } => lit,
        Block::RedstoneWallTorch { lit, .. } => lit,
        Block::Repeater(repeater) => &mut repeater.powered,
        Block::Lever { powered, .. } => powered,
        Block::StoneButton { powered, .. } => powered,
        Block::RedstoneLamp { lit } => lit,
        Block::IronTrapdoor { powered, .. } => powered,
        Block::NoteBlock { powered, .. } => powered,
        _ => return block.get_pressure_plate_powered(),
    })
}
