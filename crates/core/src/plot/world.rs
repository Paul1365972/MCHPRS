use super::{PLOT_BLOCK_HEIGHT, PLOT_BLOCK_WIDTH, PLOT_WIDTH};
use crate::player::EntityId;
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_network::chunk::encode_block_changes;
use mchprs_network::packets::clientbound::{CBlockEntityData, ClientBoundPacket};
use mchprs_network::PlayerPacketSender;
use mchprs_redstone::sound::Sound;
use mchprs_redstone::tick_queue::{run_scheduled_ticks, TickPriority};
use mchprs_redstone::{plate, Aabb, Redstone, RedstoneWorld};
use mchprs_save_data::plot_data::{PlotData, WorldSendRate};
use mchprs_world::occupied_regions;
use mchprs_world::storage::Chunk;
use rustc_hash::FxHashMap;
use std::collections::HashMap;
use std::time::{Duration, Instant};

const SAVE_RETRY_INTERVAL: Duration = Duration::from_secs(60);

#[derive(Debug, Clone, Copy)]
pub(super) enum SaveState {
    Saved,
    Unsaved { since: Instant },
    Failed { at: Instant },
}

impl SaveState {
    pub(super) fn is_saved(self) -> bool {
        matches!(self, SaveState::Saved)
    }

    pub(super) fn is_failing(self) -> bool {
        matches!(self, SaveState::Failed { at } if at.elapsed() < SAVE_RETRY_INTERVAL)
    }

    pub(super) fn is_due(self, interval: Duration) -> bool {
        match self {
            SaveState::Saved => false,
            SaveState::Unsaved { since } => since.elapsed() >= interval,
            SaveState::Failed { at } => at.elapsed() >= SAVE_RETRY_INTERVAL,
        }
    }
}

pub struct PlotWorld {
    pub x: i32,
    pub z: i32,
    pub chunks: Vec<Chunk>,
    pub(super) redstone: Redstone,
    pub(super) entity_boxes: Vec<Aabb>,
    pub packet_senders: HashMap<EntityId, PlayerPacketSender>,
    pub(super) world_send_rate: WorldSendRate,
    pub(super) pending_block_entities: FxHashMap<BlockPos, CBlockEntityData>,
    pub(super) pending_sounds: FxHashMap<BlockPos, Sound>,
    pub(super) save_state: SaveState,
}

impl PlotWorld {
    fn get_chunk_index_for_chunk(&self, chunk_x: i32, chunk_z: i32) -> Option<usize> {
        chunk_index(self.x, self.z, chunk_x, chunk_z)
    }

    fn get_chunk_index_for_block(&self, block_x: i32, block_z: i32) -> Option<usize> {
        chunk_index(self.x, self.z, block_x >> 4, block_z >> 4)
    }

    pub(super) fn copy_blocks(&self) -> PlotBlocks {
        PlotBlocks {
            x: self.x,
            z: self.z,
            chunks: self.chunks.clone(),
        }
    }

    pub fn region(&self) -> Region {
        const W: i32 = PLOT_BLOCK_WIDTH;
        Region::new(
            BlockPos::new(self.x * W, 0, self.z * W),
            BlockPos::new(
                (self.x + 1) * W - 1,
                PLOT_BLOCK_HEIGHT - 1,
                (self.z + 1) * W - 1,
            ),
        )
    }

    pub fn chunk(&self, x: i32, z: i32) -> Option<&Chunk> {
        self.chunks.get(self.get_chunk_index_for_chunk(x, z)?)
    }

    pub fn flush_block_changes(&mut self) {
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
    }

    pub(super) fn mark_unsaved(&mut self) {
        if self.save_state.is_saved() {
            self.save_state = SaveState::Unsaved {
                since: Instant::now(),
            };
        }
    }

    pub(super) fn sign_data(&self, pos: BlockPos) -> Option<CBlockEntityData> {
        let blank;
        let sign = match self.block_entity(pos) {
            Some(sign @ BlockEntity::Sign(_)) => sign,
            _ => {
                blank = BlockEntity::Sign(Box::default());
                &blank
            }
        };
        let nbt = sign.to_nbt(true)?;
        Some(CBlockEntityData {
            x: pos.x,
            y: pos.y,
            z: pos.z,
            ty: sign.ty(),
            nbt,
        })
    }
}

impl PlotWorld {
    pub(super) fn tick(&mut self) {
        if !self.redstone.ticks.entries().is_empty() {
            self.mark_unsaved();
        }
        run_scheduled_ticks(self);
        self.press_touched_plates();
    }

    pub(super) fn is_settled(&self) -> bool {
        self.redstone.is_idle()
    }

    pub(super) fn press_touched_plates(&mut self) -> bool {
        let boxes = self.entity_boxes.clone();
        let mut pressed = false;
        for pos in boxes.iter().flat_map(Aabb::block_positions) {
            pressed |= plate::entity_inside(self, pos);
        }
        pressed
    }
}

impl BlockView for PlotWorld {
    fn block(&self, pos: BlockPos) -> Block {
        block_in(&self.chunks, self.x, self.z, pos)
    }

    fn block_entity(&self, pos: BlockPos) -> Option<&BlockEntity> {
        block_entity_in(&self.chunks, self.x, self.z, pos)
    }

    fn occupied_regions(&self, region: Region) -> Vec<Region> {
        occupied_regions(region, |x, z| self.chunk(x, z))
    }
}

impl BlockStore for PlotWorld {
    fn set_block(&mut self, pos: BlockPos, block: Block) -> bool {
        let chunk_index = match self.get_chunk_index_for_block(pos.x, pos.z) {
            Some(idx) => idx,
            None => return false,
        };

        if pos.y >= PLOT_BLOCK_HEIGHT || pos.y < 0 {
            return false;
        }

        let chunk = &mut self.chunks[chunk_index];
        let changed = chunk.set_block(
            (pos.x & 0xF) as u32,
            pos.y as u32,
            (pos.z & 0xF) as u32,
            block.get_id(),
        );
        if changed {
            self.mark_unsaved();
        }
        changed
    }

    fn set_block_entity(&mut self, pos: BlockPos, block_entity: Option<BlockEntity>) {
        self.mark_unsaved();
        let Some(block_entity) = block_entity else {
            let had_sign = matches!(self.block_entity(pos), Some(BlockEntity::Sign(_)));
            self.pending_block_entities.remove(&pos);
            let chunk_index = match self.get_chunk_index_for_block(pos.x, pos.z) {
                Some(idx) => idx,
                None => return,
            };
            let chunk = &mut self.chunks[chunk_index];
            chunk.delete_block_entity(BlockPos::new(pos.x & 0xF, pos.y, pos.z & 0xF));
            let block = self.block(pos);
            if had_sign
                && block.is_any_sign()
                && let Some(blank) = self.sign_data(pos)
            {
                self.pending_block_entities.insert(pos, blank);
            }
            return;
        };
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
}

impl RedstoneWorld for PlotWorld {
    fn schedule_tick(&mut self, pos: BlockPos, delay: u32, priority: TickPriority, block: Block) {
        self.redstone.ticks.schedule(pos, delay, priority, block);
        self.mark_unsaved();
    }

    fn redstone(&mut self) -> &mut Redstone {
        &mut self.redstone
    }

    fn has_entity_in(&self, area: Aabb) -> bool {
        self.entity_boxes
            .iter()
            .any(|entity| entity.intersects(&area))
    }

    fn play_sound(&mut self, pos: BlockPos, sound: Sound) {
        if self.world_send_rate.0 != 0.0 {
            self.pending_sounds.insert(pos, sound);
        }
    }
}

#[derive(Clone)]
pub(super) struct PlotBlocks {
    x: i32,
    z: i32,
    pub(super) chunks: Vec<Chunk>,
}

impl BlockView for PlotBlocks {
    fn block(&self, pos: BlockPos) -> Block {
        block_in(&self.chunks, self.x, self.z, pos)
    }

    fn block_entity(&self, pos: BlockPos) -> Option<&BlockEntity> {
        block_entity_in(&self.chunks, self.x, self.z, pos)
    }

    fn occupied_regions(&self, region: Region) -> Vec<Region> {
        occupied_regions(region, |x, z| {
            self.chunks.get(chunk_index(self.x, self.z, x, z)?)
        })
    }
}

fn chunk_index(plot_x: i32, plot_z: i32, chunk_x: i32, chunk_z: i32) -> Option<usize> {
    let local_x = chunk_x - plot_x * PLOT_WIDTH;
    let local_z = chunk_z - plot_z * PLOT_WIDTH;
    if !(0..PLOT_WIDTH).contains(&local_x) || !(0..PLOT_WIDTH).contains(&local_z) {
        return None;
    }
    Some(PlotData::chunk_idx(PLOT_WIDTH, local_x, local_z))
}

fn block_in(chunks: &[Chunk], plot_x: i32, plot_z: i32, pos: BlockPos) -> Block {
    let Some(index) = chunk_index(plot_x, plot_z, pos.x >> 4, pos.z >> 4) else {
        return Block::Air;
    };
    let chunk = &chunks[index];
    let id = chunk.get_block((pos.x & 0xF) as u32, pos.y as u32, (pos.z & 0xF) as u32);
    Block::from_id(id).expect("plot chunks hold known block states")
}

fn block_entity_in(
    chunks: &[Chunk],
    plot_x: i32,
    plot_z: i32,
    pos: BlockPos,
) -> Option<&BlockEntity> {
    let chunk = &chunks[chunk_index(plot_x, plot_z, pos.x >> 4, pos.z >> 4)?];
    chunk.get_block_entity(BlockPos::new(pos.x & 0xF, pos.y, pos.z & 0xF))
}
