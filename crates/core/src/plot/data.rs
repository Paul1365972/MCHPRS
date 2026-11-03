use super::{Plot, PlotWorld, PLOT_WIDTH};
use anyhow::{Context, Result};
use mchprs_save_data::plot_data::{ChunkData, PlotData, Tps, WorldSendRate};
use std::collections::HashMap;
use std::path::Path;

pub fn load_plot(path: impl AsRef<Path>) -> Result<PlotData<'static>> {
    let path = path.as_ref();
    PlotData::load_from_file(path)
        .with_context(|| format!("error loading plot save file at {}", path.display()))
}

pub fn empty_plot() -> Result<PlotData<'static>> {
    let template_path = Path::new("./world/plots/pTEMPLATE");
    if template_path.exists() {
        PlotData::load_from_file(template_path).context("Failed to load world/plots/pTEMPLATE")
    } else {
        let mut chunks = Vec::new();
        for chunk_x in 0..PLOT_WIDTH {
            for chunk_z in 0..PLOT_WIDTH {
                chunks.push(Plot::generate_chunk(8, chunk_x, chunk_z));
            }
        }
        let mut world = PlotWorld {
            x: 0,
            z: 0,
            chunks,
            ticks: Default::default(),
            packet_senders: HashMap::new(),
            world_send_rate: WorldSendRate::default(),
            pending_block_entities: Default::default(),
            pending_sounds: Default::default(),
        };
        let chunk_data: Vec<ChunkData<'_>> = world
            .chunks
            .iter_mut()
            .map(|chunk| ChunkData::new(chunk).into_owned())
            .collect();
        Ok(PlotData {
            tps: Tps::Limited(10.0),
            world_send_rate: WorldSendRate::default(),
            chunk_data,
            pending_ticks: Vec::new(),
        })
    }
}
