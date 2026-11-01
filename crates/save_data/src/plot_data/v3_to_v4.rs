//! Migration from version 3 to version 4, which saves the block each pending tick is for

use super::{ChunkData, PlotData, PlotLoadError, SavedTick, Tps, WorldSendRate, PLOT_MAGIC};
use byteorder::{LittleEndian, ReadBytesExt};
use mchprs_blocks::BlockPos;
use mchprs_redstone::tick_queue::TickPriority;
use mchprs_world::storage::Chunk;
use serde::Deserialize;
use std::fs::File;
use std::io::Read;
use std::path::Path;
use tracing::debug;

#[derive(Deserialize)]
pub struct TickEntryV3 {
    ticks_left: u32,
    tick_priority: TickPriority,
    pos: BlockPos,
}

#[derive(Deserialize)]
pub struct PlotDataV3 {
    pub tps: Tps,
    pub world_send_rate: WorldSendRate,
    pub chunk_data: Vec<ChunkData>,
    pub pending_ticks: Vec<TickEntryV3>,
}

impl PlotDataV3 {
    pub fn upgrade(self) -> PlotData {
        let plot_width = self.chunk_data.len().isqrt() as i32;
        let chunks: Vec<Chunk> = self
            .chunk_data
            .iter()
            .map(|chunk| chunk.clone().load(0, 0))
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
                    position: tick.pos,
                    block_id: chunk.get_block(
                        (tick.pos.x & 0xF) as u32,
                        tick.pos.y as u32,
                        (tick.pos.z & 0xF) as u32,
                    ),
                })
            })
            .collect();
        PlotData {
            tps: self.tps,
            world_send_rate: self.world_send_rate,
            chunk_data: self.chunk_data,
            pending_ticks,
        }
    }
}

pub fn convert_v3_to_v4(path: impl AsRef<Path>) -> Result<Option<PlotData>, PlotLoadError> {
    let mut file = File::open(&path)?;

    let mut magic = [0; 8];
    file.read_exact(&mut magic)?;
    if &magic != PLOT_MAGIC {
        return Ok(None);
    }

    let version = file.read_u32::<LittleEndian>()?;
    if version != 3 {
        return Ok(None);
    }

    let mut buf = Vec::new();
    file.read_to_end(&mut buf)?;
    let old_data: PlotDataV3 = bincode::deserialize(&buf)?;

    debug!("Successfully converted plot data from version 3 to version 4");
    Ok(Some(old_data.upgrade()))
}
