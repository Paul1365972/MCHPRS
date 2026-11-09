use super::*;
use anyhow::{ensure, Context, Result};
use mchprs_redstone::tick_queue::TickEntry;
use mchprs_save_data::plot_data::SavedTick;
use std::collections::HashMap;

// TODO: where to put this?
pub fn sleep_time_for_tps(tps: Tps) -> Duration {
    match tps {
        Tps::Limited(tps) => {
            if tps > 10.0 {
                Duration::from_secs_f64(1.0 / f64::from(tps))
            } else {
                Duration::from_millis(50)
            }
        }
        Tps::Unlimited => Duration::ZERO,
    }
}

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
        channels: PlotChannels,
        always_running: bool,
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
            packet_senders: HashMap::new(),
            world_send_rate: plot_data.world_send_rate,
            pending_block_entities: FxHashMap::default(),
            pending_sounds: Vec::new(),
        };
        let tps = plot_data.tps;
        Plot {
            last_player_time: Instant::now(),
            last_update_time: Instant::now(),
            last_world_send_time: Instant::now(),
            lag_time: Duration::new(0, 0),
            sleep_time: sleep_time_for_tps(tps),
            last_nspt: None,
            message_receiver: channels.broadcasts,
            message_sender: channels.server,
            priv_message_receiver: channels.private,
            shared,
            players: Vec::new(),
            locked_players: HashSet::new(),
            running: true,
            tps,
            always_running,
            redpiler: Default::default(),
            timings: TimingsMonitor::new(tps),
            scoreboard: Default::default(),
            world,
            last_autosave_time: Instant::now(),
        }
    }

    pub(super) fn load(
        x: i32,
        z: i32,
        channels: PlotChannels,
        always_running: bool,
        shared: Arc<Shared>,
    ) -> Result<Plot, (Error, Sender<Message>)> {
        let plot_path = plot_path(x, z);
        Ok(if Path::new(&plot_path).exists() {
            match load_plot(plot_path) {
                Ok(data) => Plot::from_data(data, x, z, channels, always_running, shared),
                Err(err) => {
                    return Result::Err((
                        err.context(format!("error loading plot {},{}", x, z)),
                        channels.server,
                    ))
                }
            }
        } else {
            Plot::from_data(
                shared.empty_plot.clone(),
                x,
                z,
                channels,
                always_running,
                shared,
            )
        })
    }

    pub(super) fn save(&mut self) {
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
        data.save_to_file(plot_path(world.x, world.z)).unwrap();

        self.reset_timings();
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
