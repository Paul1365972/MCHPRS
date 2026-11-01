use super::PLOT_SCALE;
use super::{PLOT_BLOCK_HEIGHT, PLOT_BLOCK_WIDTH, PLOT_WIDTH};
use crate::player::EntityId;
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::BlockPos;
use mchprs_network::chunk::encode_block_changes;
use mchprs_network::packets::clientbound::{CBlockEntityData, CSoundEffect, ClientBoundPacket};
use mchprs_network::PlayerPacketSender;
use mchprs_save_data::plot_data::WorldSendRate;
use mchprs_world::storage::Chunk;
use mchprs_world::{TickEntry, TickPriority, World};
use rustc_hash::FxHashMap;
use std::collections::HashMap;

pub struct PlotWorld {
    pub x: i32,
    pub z: i32,
    pub chunks: Vec<Chunk>,
    pub to_be_ticked: Vec<TickEntry>,
    pub packet_senders: HashMap<EntityId, PlayerPacketSender>,
    pub(super) world_send_rate: WorldSendRate,
    pub(super) pending_block_entities: FxHashMap<BlockPos, CBlockEntityData>,
    pub(super) pending_sounds: FxHashMap<BlockPos, CSoundEffect>,
}

impl PlotWorld {
    pub(super) fn get_chunk_index_for_chunk(&self, chunk_x: i32, chunk_z: i32) -> usize {
        let local_x = chunk_x - self.x * PLOT_WIDTH;
        let local_z = chunk_z - self.z * PLOT_WIDTH;
        (local_x * PLOT_WIDTH + local_z).unsigned_abs() as usize
    }

    fn get_chunk_index_for_block(&self, block_x: i32, block_z: i32) -> Option<usize> {
        let chunk_x = (block_x - (self.x * PLOT_BLOCK_WIDTH)) >> 4;
        let chunk_z = (block_z - (self.z * PLOT_BLOCK_WIDTH)) >> 4;
        if !(0..PLOT_WIDTH).contains(&chunk_x) || !(0..PLOT_WIDTH).contains(&chunk_z) {
            return None;
        }
        Some(((chunk_x << PLOT_SCALE) + chunk_z).unsigned_abs() as usize)
    }

    pub fn get_corners(&self) -> (BlockPos, BlockPos) {
        const W: i32 = PLOT_BLOCK_WIDTH;
        let first_pos = BlockPos::new(self.x * W, 0, self.z * W);
        let second_pos = BlockPos::new(
            (self.x + 1) * W - 1,
            PLOT_BLOCK_HEIGHT - 1,
            (self.z + 1) * W - 1,
        );
        (first_pos, second_pos)
    }
}

impl World for PlotWorld {
    /// Sets a block in storage. Returns true if a block was changed.
    fn set_block_raw(&mut self, pos: BlockPos, block: u32) -> bool {
        let chunk_index = match self.get_chunk_index_for_block(pos.x, pos.z) {
            Some(idx) => idx,
            None => return false,
        };

        // Check to see if block is within height limit
        if pos.y >= PLOT_BLOCK_HEIGHT || pos.y < 0 {
            return false;
        }

        let chunk = &mut self.chunks[chunk_index];
        chunk.set_block(
            (pos.x & 0xF) as u32,
            pos.y as u32,
            (pos.z & 0xF) as u32,
            block,
        )
    }

    /// Returns the block state id of the block at `pos`
    fn get_block_raw(&self, pos: BlockPos) -> u32 {
        let chunk_index = match self.get_chunk_index_for_block(pos.x, pos.z) {
            Some(idx) => idx,
            None => return 0,
        };
        let chunk = &self.chunks[chunk_index];
        chunk.get_block((pos.x & 0xF) as u32, pos.y as u32, (pos.z & 0xF) as u32)
    }

    fn delete_block_entity(&mut self, pos: BlockPos) {
        self.pending_block_entities.remove(&pos);
        let chunk_index = match self.get_chunk_index_for_block(pos.x, pos.z) {
            Some(idx) => idx,
            None => return,
        };
        let chunk = &mut self.chunks[chunk_index];
        chunk.delete_block_entity(BlockPos::new(pos.x & 0xF, pos.y, pos.z & 0xF));
    }

    fn get_block_entity(&self, pos: BlockPos) -> Option<&BlockEntity> {
        let chunk_index = self.get_chunk_index_for_block(pos.x, pos.z)?;
        let chunk = &self.chunks[chunk_index];
        chunk.get_block_entity(BlockPos::new(pos.x & 0xF, pos.y, pos.z & 0xF))
    }

    fn set_block_entity(&mut self, pos: BlockPos, block_entity: BlockEntity) {
        if !(0..PLOT_BLOCK_HEIGHT).contains(&pos.y) {
            return;
        }
        let chunk_index = match self.get_chunk_index_for_block(pos.x, pos.z) {
            Some(idx) => idx,
            None => return,
        };
        if let Some(nbt) = block_entity.to_nbt(true) {
            let block_entity_data = CBlockEntityData {
                x: pos.x,
                y: pos.y,
                z: pos.z,
                // For now the only nbt we send to the client is sign data
                ty: block_entity.ty(),
                nbt,
            };
            self.pending_block_entities.insert(pos, block_entity_data);
        } else {
            self.pending_block_entities.remove(&pos);
        }
        let chunk = &mut self.chunks[chunk_index];
        chunk.set_block_entity(BlockPos::new(pos.x & 0xF, pos.y, pos.z & 0xF), block_entity);
    }

    fn get_chunk(&self, x: i32, z: i32) -> Option<&Chunk> {
        self.chunks.get(self.get_chunk_index_for_chunk(x, z))
    }

    fn get_chunk_mut(&mut self, x: i32, z: i32) -> Option<&mut Chunk> {
        let chunk_idx = self.get_chunk_index_for_chunk(x, z);
        self.chunks.get_mut(chunk_idx)
    }

    fn schedule_tick(&mut self, pos: BlockPos, delay: u32, priority: TickPriority) {
        self.to_be_ticked.push(TickEntry {
            pos,
            ticks_left: delay,
            tick_priority: priority,
        });
    }

    fn pending_tick_at(&mut self, pos: BlockPos) -> bool {
        self.to_be_ticked.iter().any(|e| e.pos == pos)
    }

    fn play_sound(
        &mut self,
        pos: BlockPos,
        sound_id: i32,
        sound_category: i32,
        volume: f32,
        pitch: f32,
    ) {
        if self.world_send_rate.0 == 0.0 {
            return;
        }
        // FIXME: We do not know the players location here, so we send the sound packet to all
        // players A notchian server would only send to players in hearing distance
        // (volume.clamp(0.0, 1.0) * 16.0)
        let sound_effect_data = CSoundEffect {
            sound_id: sound_id + 1,
            sound_name: None,
            has_fixed_range: None,
            range: None,
            sound_category,
            x: pos.x * 8 + 4,
            y: pos.y * 8 + 4,
            z: pos.z * 8 + 4,
            volume,
            pitch,
            // FIXME: How do we decide this?
            seed: 0,
        };
        self.pending_sounds.insert(pos, sound_effect_data);
    }

    fn flush_block_changes(&mut self) {
        for packet in self.chunks.iter_mut().flat_map(encode_block_changes) {
            for packet_sender in self.packet_senders.values() {
                packet_sender.send_packet(&packet);
            }
        }
        for (_, block_entity) in self.pending_block_entities.drain() {
            let encoded = block_entity.encode();
            for packet_sender in self.packet_senders.values() {
                packet_sender.send_packet(&encoded);
            }
        }
        for (_, sound) in self.pending_sounds.drain() {
            let encoded = sound.encode();
            for packet_sender in self.packet_senders.values() {
                packet_sender.send_packet(&encoded);
            }
        }
    }
}
