use super::advance::Advances;
use super::scoreboard::Scoreboard;
use super::send_schedule::SendSchedule;
use super::simulation::PlotSimulation;
use super::tick_history::TickHistory;
use super::world::{PlotWorld, SaveState};
use super::{Plot, NUM_CHUNKS, PLOT_BLOCK_WIDTH, PLOT_SCALE, PLOT_SECTIONS, PLOT_WIDTH};
use crate::server::{BroadcastMessage, Message};
use crate::shared::Shared;
use anyhow::{ensure, Context, Error, Result};
use bus::BusReader;
use mchprs_blocks::blocks::Block;
use mchprs_redstone::tick_queue::{TickEntry, TickQueue};
use mchprs_redstone::{noteblock, Redstone};
use mchprs_save_data::plot_data::{ChunkData, PlotData, SavedTick, Tps, WorldSendRate};
use mchprs_simulation::schedule::Schedule;
use mchprs_world::storage::Chunk;
use rustc_hash::FxHashMap;
use std::collections::{HashMap, HashSet};
use std::path::Path;
use std::sync::mpsc::Sender;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tracing::{debug, error};

pub fn load_plot(path: impl AsRef<Path>) -> Result<PlotData<'static>> {
    let path = path.as_ref();
    let data = PlotData::load_from_file(path)
        .with_context(|| format!("error loading plot save file at {}", path.display()))?;
    let chunk_count = data.chunk_data.len();
    ensure!(
        chunk_count == NUM_CHUNKS,
        "{} has {chunk_count} chunks, but plot scale {PLOT_SCALE} needs {NUM_CHUNKS}",
        path.display()
    );
    Ok(data)
}

pub fn empty_plot() -> Result<PlotData<'static>> {
    let template_path = Path::new("./world/plots/pTEMPLATE");
    if template_path.exists() {
        load_plot(template_path)
    } else {
        let chunk_data = (0..NUM_CHUNKS)
            .map(|idx| {
                let (chunk_x, chunk_z) = PlotData::chunk_pos(PLOT_WIDTH, idx);
                let mut chunk = Plot::generate_chunk(8, chunk_x, chunk_z);
                ChunkData::new(&mut chunk).into_owned()
            })
            .collect();
        Ok(PlotData {
            tps: Tps::Limited(10.0),
            world_send_rate: WorldSendRate::default(),
            chunk_data,
            pending_ticks: Vec::new(),
        })
    }
}

impl Plot {
    fn generate_chunk(layers: i32, x: i32, z: i32) -> Chunk {
        let mut chunk = Chunk::empty(x, z, PLOT_SECTIONS);

        for ry in 0..layers {
            for rx in 0..16 {
                for rz in 0..16 {
                    let block_x = (x << 4) | rx;
                    let block_z = (z << 4) | rz;

                    let block = if block_x % PLOT_BLOCK_WIDTH == 0
                        || block_z % PLOT_BLOCK_WIDTH == 0
                        || (block_x + 1) % PLOT_BLOCK_WIDTH == 0
                        || (block_z + 1) % PLOT_BLOCK_WIDTH == 0
                    {
                        Block::StoneBricks
                    } else {
                        Block::Sandstone
                    };
                    chunk.set_block(rx as u32, ry as u32, rz as u32, block.get_id());
                }
            }
        }
        chunk
    }

    fn from_data(
        plot_data: PlotData<'_>,
        x: i32,
        z: i32,
        broadcasts: BusReader<BroadcastMessage>,
        server: Sender<Message>,
        shared: Arc<Shared>,
    ) -> Plot {
        let world = PlotWorld {
            x,
            z,
            chunks: load_chunks(plot_data.chunk_data, x, z),
            redstone: Redstone::new(TickQueue::new(
                plot_data
                    .pending_ticks
                    .into_iter()
                    .map(TickEntry::from)
                    .collect(),
            )),
            entity_boxes: Vec::new(),
            packet_senders: HashMap::new(),
            world_send_rate: plot_data.world_send_rate,
            pending_block_entities: FxHashMap::default(),
            pending_sounds: FxHashMap::default(),
            save_state: SaveState::Saved,
        };
        let tps = plot_data.tps;
        let now = Instant::now();
        Plot {
            last_player_time: now,
            last_scoreboard_time: now,
            simulation: PlotSimulation::World(Schedule::default()),
            send_schedule: SendSchedule::new(plot_data.world_send_rate, now),
            message_receiver: broadcasts,
            message_sender: server,
            players: Vec::new(),
            locked_players: HashSet::new(),
            reported_idle: false,
            tps,
            advances: Advances::default(),
            ticks: 0,
            tick_history: TickHistory::new(now),
            scoreboard: Scoreboard::new(x, z),
            shared,
            world,
        }
    }

    pub(super) fn load(
        x: i32,
        z: i32,
        broadcasts: BusReader<BroadcastMessage>,
        server: Sender<Message>,
        shared: Arc<Shared>,
    ) -> Result<Plot, Error> {
        let plot_path = plot_path(x, z);
        let mut plot = if Path::new(&plot_path).exists() {
            let data =
                load_plot(plot_path).with_context(|| format!("error loading plot {},{}", x, z))?;
            Plot::from_data(data, x, z, broadcasts, server, shared)
        } else {
            let data = shared.empty_plot.clone();
            let mut plot = Plot::from_data(data, x, z, broadcasts, server, shared);
            plot.world.mark_unsaved();
            plot
        };
        plot.recheck_pressure_plates();
        Ok(plot)
    }

    fn needs_saving(&self) -> bool {
        self.simulation.is_editable() && !self.world.save_state.is_saved()
    }

    pub(super) fn save_for_unload(&mut self) {
        if self.needs_saving() {
            for chunk in &mut self.world.chunks {
                chunk.flush();
                chunk.compress();
            }
        }
        self.save();
    }

    pub(super) fn autosave_if_due(&mut self) {
        let interval = Duration::from_secs(self.shared.config.autosave_interval_seconds);
        if !interval.is_zero() && self.world.save_state.is_due(interval) {
            self.save();
        }
    }

    pub(super) fn save(&mut self) {
        if !self.needs_saving() {
            return;
        }
        // A saved plot holds no block events.
        noteblock::run_block_events(&mut self.world);
        let start = Instant::now();
        let world = &mut self.world;
        let chunk_data: Vec<ChunkData<'_>> = world.chunks.iter_mut().map(ChunkData::new).collect();
        let data = PlotData {
            tps: self.tps,
            world_send_rate: world.world_send_rate,
            chunk_data,
            pending_ticks: world
                .redstone
                .ticks
                .entries()
                .iter()
                .map(SavedTick::from)
                .collect(),
        };
        match data.save_to_file(plot_path(world.x, world.z)) {
            Ok(()) => {
                world.save_state = SaveState::Saved;
                debug!(
                    "Saved plot ({}, {}) in {:?}",
                    world.x,
                    world.z,
                    start.elapsed()
                );
            }
            Err(error) => {
                world.save_state = SaveState::Failed { at: Instant::now() };
                error!("Failed to save plot ({}, {}): {error}", world.x, world.z);
            }
        }
    }
}

fn plot_path(x: i32, z: i32) -> String {
    format!("./world/plots/p{},{}", x, z)
}

fn load_chunks(chunk_data: Vec<ChunkData<'_>>, x: i32, z: i32) -> Vec<Chunk> {
    let chunk_x_offset = x << PLOT_SCALE;
    let chunk_z_offset = z << PLOT_SCALE;
    chunk_data
        .into_iter()
        .enumerate()
        .map(|(i, c)| {
            let (local_x, local_z) = PlotData::chunk_pos(PLOT_WIDTH, i);
            c.load(chunk_x_offset + local_x, chunk_z_offset + local_z)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::PLOT_SECTIONS;
    use mchprs_save_data::plot_data::ChunkData;
    use mchprs_world::storage::Chunk;

    #[test]
    fn chunk_save_and_load_test() {
        let mut chunk = Chunk::empty(1, 1, PLOT_SECTIONS);
        chunk.set_block(13, 63, 12, 332);
        chunk.set_block(13, 62, 12, 331);
        let chunk_data = ChunkData::new(&mut chunk);
        let loaded_chunk = chunk_data.load(1, 1);
        assert_eq!(loaded_chunk.get_block(13, 63, 12), 332);
        assert_eq!(loaded_chunk.get_block(13, 62, 12), 331);
        assert_eq!(loaded_chunk.get_block(13, 64, 12), 0);
    }
}
