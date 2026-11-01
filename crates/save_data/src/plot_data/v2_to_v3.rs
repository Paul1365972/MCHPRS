//! Migration from version 2 to version 3, which turned Tps and WorldSendRate from u32 into f32

use super::{ChunkData, PlotData, Tps, WorldSendRate};
use mchprs_world::TickEntry;
use serde::Deserialize;

#[derive(Deserialize)]
enum TpsV2 {
    Limited(u32),
    Unlimited,
}

#[derive(Deserialize)]
struct WorldSendRateV2(u32);

#[derive(Deserialize)]
pub struct PlotDataV2 {
    tps: TpsV2,
    world_send_rate: WorldSendRateV2,
    chunk_data: Vec<ChunkData>,
    pending_ticks: Vec<TickEntry>,
}

impl PlotDataV2 {
    pub fn upgrade(self) -> PlotData {
        let tps = match self.tps {
            TpsV2::Limited(val) => Tps::Limited(val as f32),
            TpsV2::Unlimited => Tps::Unlimited,
        };
        PlotData {
            tps,
            world_send_rate: WorldSendRate(self.world_send_rate.0 as f32),
            chunk_data: self.chunk_data,
            pending_ticks: self.pending_ticks,
        }
    }
}
