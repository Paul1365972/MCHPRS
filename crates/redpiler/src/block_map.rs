use crate::backend::Batch;
use crate::compile_graph::NodeType;
use crate::engine::{Input, NodeChanges, NodeOutput, PendingTick};
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
    outputs: Box<[NodeOutput]>,
    nodes_by_position: FxHashMap<BlockPos, NodeId>,
}

impl BlockMap {
    pub fn new(netlist: &Netlist) -> BlockMap {
        let mut nodes_by_position = FxHashMap::default();
        let mut blocks = Vec::with_capacity(netlist.len());
        let mut types = Vec::with_capacity(netlist.len());
        let mut outputs = Vec::with_capacity(netlist.len());
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
            outputs.push(NodeOutput::from(node.state));
        }
        BlockMap {
            blocks: blocks.into_boxed_slice(),
            types: types.into_boxed_slice(),
            outputs: outputs.into_boxed_slice(),
            nodes_by_position,
        }
    }

    pub fn node_at(&self, pos: BlockPos) -> Option<NodeId> {
        self.nodes_by_position.get(&pos).copied()
    }

    pub fn node_accepting(&self, pos: BlockPos, input: Input) -> Option<NodeId> {
        self.node_at(pos)
            .filter(|node| input.is_accepted_by(&self.types[node.index()]))
    }

    pub fn tick_entries(&self, ticks: &[PendingTick]) -> Vec<TickEntry> {
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

    pub fn pending_ticks(&self, entries: &[TickEntry]) -> Vec<PendingTick> {
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
        for &(node, output) in &changes.outputs {
            self.record_output(node, output, &mut out);
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

    fn record_output(&mut self, node: NodeId, output: NodeOutput, out: &mut Batch) {
        if self.outputs[node.index()] == output {
            return;
        }
        self.outputs[node.index()] = output;
        let is_comparator = matches!(self.types[node.index()], NodeType::Comparator { .. });
        for (pos, block) in &mut self.blocks[node.index()] {
            set_block_output(block, output);
            out.blocks.push((*pos, *block));
            if is_comparator {
                let block_entity = BlockEntity::Comparator {
                    output_strength: output.power.get(),
                };
                out.block_entities.push((*pos, block_entity));
            }
        }
    }
}

fn set_block_output(block: &mut Block, output: NodeOutput) {
    if let Some(powered) = block_powered_mut(block) {
        *powered = output.is_powered();
    }
    match block {
        Block::IronTrapdoor { open, .. } => *open = output.is_powered(),
        Block::RedstoneWire(wire) => wire.power = output.power.get(),
        Block::Repeater(repeater) => repeater.locked = output.repeater_locked,
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
