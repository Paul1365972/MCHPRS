//! Migration from version 3 to version 4, which saves the block each pending tick is for

use super::v4_to_v5::{ChunkDataV4, PlotDataV4};
use super::{ChunkData, SavedTick, Tps, WorldSendRate};
use mchprs_blocks::BlockPos;
use mchprs_redstone::tick_queue::TickPriority;
use mchprs_world::storage::Chunk;
use serde::Deserialize;
use std::borrow::Cow;

#[derive(Debug, Deserialize)]
pub struct TickEntryV3 {
    ticks_left: u32,
    tick_priority: TickPriority,
    pos: BlockPos,
}

#[derive(Debug, Deserialize)]
pub struct PlotDataV3 {
    pub tps: Tps,
    pub world_send_rate: WorldSendRate,
    pub chunk_data: Vec<ChunkDataV4>,
    pub pending_ticks: Vec<TickEntryV3>,
}

impl PlotDataV3 {
    pub fn upgrade(self) -> PlotDataV4 {
        let plot_width = self.chunk_data.len().isqrt() as i32;
        let chunks: Vec<Chunk> = self
            .chunk_data
            .iter()
            .map(|chunk| {
                let sections = chunk.sections.clone();
                ChunkData {
                    sections,
                    block_entities: Cow::default(),
                }
                .load(0, 0)
            })
            .collect();
        let pending_ticks = self
            .pending_ticks
            .into_iter()
            .filter_map(|tick| {
                let local_chunk_x = (tick.pos.x >> 4).checked_rem_euclid(plot_width)?;
                let local_chunk_z = (tick.pos.z >> 4).checked_rem_euclid(plot_width)?;
                let chunk = &chunks[(local_chunk_x * plot_width + local_chunk_z) as usize];
                Some(SavedTick {
                    ticks_left: tick.ticks_left,
                    priority: tick.tick_priority,
                    pos: tick.pos,
                    block_id: chunk.get_block(
                        (tick.pos.x & 0xF) as u32,
                        tick.pos.y as u32,
                        (tick.pos.z & 0xF) as u32,
                    ),
                })
            })
            .collect();
        PlotDataV4 {
            tps: self.tps,
            world_send_rate: self.world_send_rate,
            chunk_data: self.chunk_data,
            pending_ticks,
        }
    }
}
