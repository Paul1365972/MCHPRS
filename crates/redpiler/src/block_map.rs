use crate::circuit::{Circuit, NodeId};
use crate::engine::{Changes, Drive, Input, Snapshot};
use crate::graph::{NodeKind, NodeState};
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::blocks::{Block, Instrument};
use mchprs_blocks::BlockPos;
use mchprs_redstone::RedstoneWorld;
use smallvec::SmallVec;
use std::mem;

type NodeBlocks = SmallVec<[(BlockPos, Block); 1]>;

#[derive(Debug, Clone, Default)]
pub struct BlockChanges {
    pub blocks: Vec<(BlockPos, Block)>,
    pub block_entities: Vec<(BlockPos, BlockEntity)>,
    pub notes: Vec<PlayedNote>,
}

impl BlockChanges {
    pub fn is_empty(&self) -> bool {
        self.blocks.is_empty() && self.block_entities.is_empty() && self.notes.is_empty()
    }

    pub fn apply(&self, world: &mut impl RedstoneWorld) {
        for &(pos, block) in &self.blocks {
            world.set_block(pos, block);
        }
        for (pos, block_entity) in &self.block_entities {
            world.set_block_entity(*pos, Some(block_entity.clone()));
        }
        for note in &self.notes {
            world.play_note(note.pos, note.instrument, note.note);
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlayedNote {
    pub pos: BlockPos,
    pub instrument: Instrument,
    pub note: u8,
    pub count: u32,
}

pub struct BlockMap {
    circuit: Circuit,
    blocks: Box<[NodeBlocks]>,
    states: Box<[NodeState]>,
    dirty: Box<[bool]>,
    dirty_nodes: Vec<NodeId>,
    plays: Box<[u32]>,
    played: Vec<NodeId>,
}

impl BlockMap {
    pub fn new(circuit: Circuit) -> BlockMap {
        BlockMap {
            blocks: circuit
                .ids()
                .map(|id| {
                    let blocks = circuit.node(id).blocks.iter();
                    let blocks = blocks.map(|&(pos, id)| (pos, Block::from_id(id)));
                    blocks.collect()
                })
                .collect(),
            states: circuit.ids().map(|id| circuit.node(id).state).collect(),
            dirty: vec![false; circuit.len()].into_boxed_slice(),
            dirty_nodes: Vec::new(),
            plays: vec![0; circuit.len()].into_boxed_slice(),
            played: Vec::new(),
            circuit,
        }
    }

    pub fn circuit(&self) -> &Circuit {
        &self.circuit
    }

    pub fn use_block(&self, pos: BlockPos) -> Option<Drive> {
        let node = self.circuit.node_at(pos)?;
        let input = match self.circuit.node(node).kind {
            NodeKind::Lever => Input::Toggle,
            NodeKind::Button { .. } => Input::Press,
            _ => return None,
        };
        self.circuit.check_input(node, input).ok()
    }

    pub fn pressure_plate(&self, pos: BlockPos, pressed: bool) -> Option<Drive> {
        let node = self.circuit.node_at(pos)?;
        if self.circuit.node(node).kind != NodeKind::PressurePlate {
            return None;
        }
        self.circuit
            .check_input(node, Input::Set(pressed.into()))
            .ok()
    }

    pub fn pressure_plates(&self) -> impl Iterator<Item = (BlockPos, bool)> + '_ {
        self.circuit
            .ids()
            .filter(|&id| {
                let node = self.circuit.node(id);
                node.kind == NodeKind::PressurePlate && node.is_input()
            })
            .flat_map(|id| {
                let pressed = self.states[id.index()].is_powered();
                self.blocks[id.index()]
                    .iter()
                    .map(move |&(pos, _)| (pos, pressed))
            })
    }

    pub fn record(&mut self, changes: &Changes) {
        for &(node, state) in &changes.states {
            self.set_state(node, state);
        }
        for &(node, count) in &changes.notes {
            let plays = &mut self.plays[node.index()];
            if *plays == 0 {
                self.played.push(node);
            }
            *plays = plays.saturating_add(count);
        }
    }

    pub fn record_snapshot(&mut self, snapshot: &Snapshot) {
        for (index, &state) in snapshot.states.iter().enumerate() {
            self.set_state(NodeId::new(index), state);
        }
    }

    /// Compares blocks rather than states, since a block can disagree with its node at compile
    /// time, like a comparator whose block entity holds a strength its block doesn't show.
    fn set_state(&mut self, node: NodeId, state: NodeState) {
        let index = node.index();
        let strength_changed = self.states[index].power != state.power;
        self.states[index] = state;
        let mut changed = false;
        for (_, block) in &mut self.blocks[index] {
            let previous = *block;
            apply_state(block, state);
            changed |=
                *block != previous || strength_changed && matches!(block, Block::Comparator(_));
        }
        if changed && !self.dirty[index] {
            self.dirty[index] = true;
            self.dirty_nodes.push(node);
        }
    }

    pub fn take_changes(&mut self) -> BlockChanges {
        let mut changes = BlockChanges::default();
        for node in self.dirty_nodes.drain(..) {
            self.dirty[node.index()] = false;
            let output_strength = self.states[node.index()].power;
            for &(pos, block) in &self.blocks[node.index()] {
                changes.blocks.push((pos, block));
                if matches!(block, Block::Comparator(_)) {
                    changes
                        .block_entities
                        .push((pos, BlockEntity::Comparator { output_strength }));
                }
            }
        }
        for node in self.played.drain(..) {
            let count = mem::take(&mut self.plays[node.index()]);
            let NodeKind::NoteBlock { instrument, note } = self.circuit.node(node).kind else {
                panic!(
                    "node {} played a note but is not a note block",
                    node.index()
                );
            };
            for &(pos, _) in &self.blocks[node.index()] {
                changes.notes.push(PlayedNote {
                    pos,
                    instrument,
                    note,
                    count,
                });
            }
        }
        changes
    }

    pub fn apply(&mut self, changes: &Changes, world: &mut impl RedstoneWorld) {
        self.record(changes);
        self.take_changes().apply(world);
    }

    pub fn finish(mut self, snapshot: &Snapshot, world: &mut impl RedstoneWorld) {
        self.record_snapshot(snapshot);
        self.take_changes().apply(world);
        for &(node, tick) in &snapshot.pending_ticks {
            for &(pos, block) in &self.blocks[node.index()] {
                world.schedule_tick(pos, tick.delay, tick.priority, block);
            }
        }
    }
}

fn apply_state(block: &mut Block, state: NodeState) {
    if let Some(powered) = powered_mut(block) {
        *powered = state.is_powered();
    }
    match block {
        Block::IronTrapdoor { open, .. } => *open = state.is_powered(),
        Block::RedstoneWire(wire) => wire.power = state.power.get(),
        Block::Repeater(repeater) => repeater.locked = state.repeater_locked,
        _ => {}
    }
}

fn powered_mut(block: &mut Block) -> Option<&mut bool> {
    Some(match block {
        Block::Comparator(comparator) => &mut comparator.powered,
        Block::RedstoneTorch { lit } => lit,
        Block::RedstoneWallTorch { lit, .. } => lit,
        Block::Repeater(repeater) => &mut repeater.powered,
        Block::Lever { powered, .. } => powered,
        Block::RedstoneLamp { lit } => lit,
        Block::IronTrapdoor { powered, .. } => powered,
        Block::NoteBlock { powered, .. } => powered,
        _ if block.button().is_some() => &mut block.button_mut()?.powered,
        _ => return block.get_pressure_plate_powered(),
    })
}
