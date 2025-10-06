use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::blocks::Block;
use mchprs_redstone::noteblock;
use mchprs_world::World;
use std::mem;
use tracing::warn;

use crate::backend::direct::node::NodeType;
use crate::backend::direct::DirectBackend;
use crate::block_powered_mut;

impl DirectBackend {
    pub(super) fn flush_note_blocks<W: World>(&mut self, world: &mut W) {
        for info in &mut self.noteblock_info {
            if mem::take(&mut info.pending) {
                for pos in info.positions.iter().copied() {
                    noteblock::play_note(world, pos, info.instrument, info.note);
                }
            }
        }
    }

    pub(super) fn flush_block_changes<W: World>(&mut self, world: &mut W) {
        for node_id in self.execution_context.drain_changes() {
            let node = &mut self.nodes[node_id];
            node.changed = false;
            for (pos, block) in &mut self.blocks[node_id.index()] {
                if let Some(powered) = block_powered_mut(block) {
                    *powered = node.powered
                }
                if let Block::IronTrapdoor { open, .. } = block {
                    *open = node.powered;
                }
                if let Block::RedstoneWire(wire) = block {
                    wire.power = node.output_power
                };
                if let Block::Repeater(repeater) = block {
                    repeater.locked = node.locked;
                }
                world.set_block(*pos, *block);
            }
        }
    }

    pub(super) fn flush_block_entities<W: World>(&mut self, world: &mut W) {
        for (i, node) in self.nodes.inner().iter().enumerate() {
            if !matches!(node.ty, NodeType::Comparator { .. }) {
                continue;
            }
            for (pos, _) in self.blocks[i].iter().copied() {
                let block_entity = BlockEntity::Comparator {
                    output_strength: node.output_power,
                };
                world.set_block_entity(pos, block_entity);
            }
        }
    }

    pub(super) fn flush_scheduled_ticks<W: World>(&mut self, world: &mut W) {
        for (delay, node_id, priority) in self.execution_context.drain_scheduled_ticks() {
            let node_blocks = &self.blocks[node_id.index()];
            if node_blocks.is_empty() {
                warn!(
                    "Cannot schedule tick for node {:?} because block information is missing",
                    node_id
                );
                continue;
            }
            for (pos, _) in node_blocks.iter().copied() {
                world.schedule_tick(pos, delay as u32, priority);
            }
        }
    }
}
