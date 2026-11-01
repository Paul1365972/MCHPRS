mod block_interaction;
#[cfg(feature = "claims")]
pub mod claims;
pub mod data;
mod monitor;
mod packet_handlers;
mod scoreboard;
#[cfg(feature = "whitelist")]
mod whitelist;
pub mod world;

use self::data::sleep_time_for_tps;
use self::scoreboard::Scoreboard;
use crate::player::{EntityId, Gamemode, PacketSender, Player, PlayerPos};
use crate::server::{BroadcastMessage, Message, PrivMessage};
use crate::shared::Shared;
use bus::BusReader;
use mchprs_blocks::{BlockFace, BlockPos};
use mchprs_network::chunk::{encode_chunk, encode_empty_chunk};
use mchprs_network::packets::clientbound::*;
use mchprs_network::PlayerPacketSender;
use mchprs_redpiler::{Compiler, CompilerOptions};
use mchprs_save_data::plot_data::{Tps, WorldSendRate};
use mchprs_world::World;
use monitor::TimingsMonitor;
use scoreboard::RedpilerState;
use std::collections::HashSet;
use std::mem;
use std::sync::mpsc::{Receiver, Sender};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};
use tracing::{debug, error, info, warn};
use world::PlotWorld;

/// The width of a plot (2^n)
pub const PLOT_SCALE: u32 = 5;

/// The width of a plot counted in chunks
pub const PLOT_WIDTH: i32 = 2i32.pow(PLOT_SCALE);
/// The plot width in blocks
pub const PLOT_BLOCK_WIDTH: i32 = PLOT_WIDTH * 16;
pub const NUM_CHUNKS: usize = PLOT_WIDTH.pow(2) as usize;

/// The height of the world in sections (Default: 24, Max: 127)
pub const PLOT_SECTIONS: usize = 24;
/// The plot height in blocks
pub const PLOT_BLOCK_HEIGHT: i32 = PLOT_SECTIONS as i32 * 16;

const ERROR_IO_ONLY: &str = "This plot cannot be interacted with while redpiler is active with `--io-only`. To stop redpiler, run `/redpiler reset`.";

const MAX_BATCH_SIZE: u32 = 50_000;

pub struct Plot {
    pub shared: Arc<Shared>,
    pub world: PlotWorld,
    pub players: Vec<Player>,
    pub redpiler: Compiler,

    // Thread communication
    message_receiver: BusReader<BroadcastMessage>,
    message_sender: Sender<Message>,
    priv_message_receiver: Receiver<PrivMessage>,

    locked_players: HashSet<EntityId>,

    // Timings
    tps: Tps,
    last_update_time: Instant,
    lag_time: Duration,
    last_nspt: Option<Duration>,
    timings: TimingsMonitor,
    /// The last time a player was in this plot
    last_player_time: Instant,
    /// The last time the world changes were sent to the player
    last_world_send_time: Instant,
    last_autosave_time: Instant,
    /// The duration we should sleep for after every update
    sleep_time: Duration,
    /// When this is false, the update loop will end and the thread will stop.
    /// This will be set to false if no players are on the plot for a certain amount of time.
    running: bool,
    /// If true, the plot will remain running even if no players are on for a long time.
    always_running: bool,

    scoreboard: Scoreboard,
}

pub struct PlotChannels {
    pub broadcasts: BusReader<BroadcastMessage>,
    pub server: Sender<Message>,
    pub private: Receiver<PrivMessage>,
}

impl Plot {
    pub fn tickn(&mut self, ticks: u64) {
        if self.redpiler.is_active() {
            self.timings.tickn(ticks);
            self.redpiler.tickn(ticks);
            return;
        }

        for _ in 0..ticks {
            self.tick();
        }
    }

    fn tick(&mut self) {
        self.timings.tick();
        if self.redpiler.is_active() {
            self.redpiler.tick();
            return;
        }

        self.world
            .to_be_ticked
            .sort_by_key(|e| (e.ticks_left, e.tick_priority));
        for pending in &mut self.world.to_be_ticked {
            pending.ticks_left = pending.ticks_left.saturating_sub(1);
        }
        while self.world.to_be_ticked.first().map_or(1, |e| e.ticks_left) == 0 {
            let entry = self.world.to_be_ticked.remove(0);
            mchprs_redstone::tick(self.world.get_block(entry.pos), &mut self.world, entry.pos);
        }
    }

    pub fn change_player_gamemode(&mut self, player_idx: usize, gamemode: Gamemode) {
        self.players[player_idx].set_gamemode(gamemode);
        let _ = self.message_sender.send(Message::PlayerUpdateGamemode(
            self.players[player_idx].uuid,
            gamemode,
        ));
    }

    pub fn teleport_player(&mut self, player_idx: usize, pos: PlayerPos) {
        let old = self.players[player_idx].pos;
        self.players[player_idx].teleport(pos);
        self.on_player_move(player_idx, old, pos);
    }

    fn on_player_move(&mut self, player_idx: usize, old: PlayerPos, new: PlayerPos) {
        let old_block = old.block_pos();
        let new_block = new.block_pos();

        if let Some(true) = self.world.get_block(old_block).get_pressure_plate_powered()
            && !self.are_players_on_block(old_block)
        {
            self.set_pressure_plate(old_block, false);
        }

        if let Some(false) = self.world.get_block(new_block).get_pressure_plate_powered()
            && self.players[player_idx].on_ground
        {
            self.set_pressure_plate(new_block, true);
        }
    }

    fn set_pressure_plate(&mut self, pos: BlockPos, new_powered: bool) {
        if self.redpiler.is_active() {
            self.redpiler.set_pressure_plate(pos, new_powered);
            self.publish_world();
            return;
        }

        let mut block = self.world.get_block(pos);
        if let Some(powered) = block.get_pressure_plate_powered() {
            *powered = new_powered;
            self.world.set_block(pos, block);
            mchprs_redstone::update_surrounding_blocks(&mut self.world, pos);
            mchprs_redstone::update_surrounding_blocks(
                &mut self.world,
                pos.offset(BlockFace::Bottom),
            );
        } else {
            warn!("Block at {} is not a pressure plate", pos);
        }
        self.publish_world();
    }

    fn are_players_on_block(&mut self, pos: BlockPos) -> bool {
        for player in &self.players {
            if player.pos.block_pos() == pos && player.on_ground {
                return true;
            }
        }
        false
    }

    fn enter_plot(&mut self, player: Player) {
        self.save();
        if self.players.is_empty() {
            self.world.pending_sounds.clear();
        }
        let spawn_player = player.spawn_packet().encode();
        let metadata = player.metadata_packet().encode();
        let entity_equipment = player.equippment_packet();
        for other_player in &mut self.players {
            other_player.client.send_packet(&spawn_player);
            other_player.client.send_packet(&metadata);
            if let Some(entity_equipment) = &entity_equipment {
                other_player.client.send_packet(&entity_equipment.encode());
            }

            let spawn_other_player = other_player.spawn_packet().encode();
            player.client.send_packet(&spawn_other_player);

            if let Some(other_entity_equipment) = other_player.equippment_packet() {
                player.client.send_packet(&other_entity_equipment.encode());
            }

            let other_metadata = other_player.metadata_packet().encode();
            player.client.send_packet(&other_metadata);
        }

        player.send_system_message(&format!(
            "Entering plot ({}, {})",
            self.world.x, self.world.z
        ));
        self.world
            .packet_senders
            .insert(player.entity_id, PlayerPacketSender::new(&player.client));
        self.scoreboard.add_player(&player);
        self.players.push(player);
        self.update_view_pos_for_player(self.players.len() - 1, true);
    }

    fn get_chunk_distance(x1: i32, z1: i32, x2: i32, z2: i32) -> u32 {
        let x = x1 - x2;
        let z = z1 - z2;
        x.abs().max(z.abs()) as u32
    }

    fn set_chunk_loaded_at_player(
        &mut self,
        player_idx: usize,
        chunk_x: i32,
        chunk_z: i32,
        was_loaded: bool,
        should_be_loaded: bool,
    ) {
        if was_loaded && !should_be_loaded {
            // let unload_chunk = CUnloadChunk { chunk_x, chunk_z }.encode();
            // self.players[player_idx].client.send_packet(&unload_chunk);
        } else if !was_loaded && should_be_loaded {
            if !Plot::chunk_in_plot_bounds(self.world.x, self.world.z, chunk_x, chunk_z) {
                self.players[player_idx]
                    .client
                    .send_packet(&encode_empty_chunk(chunk_x, chunk_z, PLOT_SECTIONS));
            } else {
                let chunk_idx = self.world.get_chunk_index_for_chunk(chunk_x, chunk_z);
                let chunk_data = encode_chunk(&self.world.chunks[chunk_idx]);
                self.players[player_idx].client.send_packet(&chunk_data);
            }
        }
    }

    pub fn update_view_pos_for_player(&mut self, player_idx: usize, force_load: bool) {
        self.publish_world();
        let view_distance = self.shared.config.view_distance as i32;
        let (chunk_x, chunk_z) = self.players[player_idx].pos.chunk_pos();
        let last_chunk_x = self.players[player_idx].last_chunk_x;
        let last_chunk_z = self.players[player_idx].last_chunk_z;

        let update_view = CSetCenterChunk { chunk_x, chunk_z }.encode();
        self.players[player_idx].client.send_packet(&update_view);

        if ((last_chunk_x - chunk_x).abs() <= view_distance * 2
            && (last_chunk_z - chunk_z).abs() <= view_distance * 2)
            && !force_load
        {
            let nx = chunk_x.min(last_chunk_x) - view_distance;
            let nz = chunk_z.min(last_chunk_z) - view_distance;
            let px = chunk_x.max(last_chunk_x) + view_distance;
            let pz = chunk_z.max(last_chunk_z) + view_distance;
            for x in nx..=px {
                for z in nz..=pz {
                    let was_loaded = Self::get_chunk_distance(x, z, last_chunk_x, last_chunk_z)
                        <= view_distance as u32;
                    let should_be_loaded =
                        Self::get_chunk_distance(x, z, chunk_x, chunk_z) <= view_distance as u32;
                    self.set_chunk_loaded_at_player(player_idx, x, z, was_loaded, should_be_loaded);
                }
            }
        } else {
            for x in last_chunk_x - view_distance..=last_chunk_x + view_distance {
                for z in last_chunk_z - view_distance..=last_chunk_z + view_distance {
                    self.set_chunk_loaded_at_player(player_idx, x, z, true, false);
                }
            }
            for x in chunk_x - view_distance..=chunk_x + view_distance {
                for z in chunk_z - view_distance..=chunk_z + view_distance {
                    self.set_chunk_loaded_at_player(player_idx, x, z, false, true);
                }
            }
        }
        self.players[player_idx].last_chunk_x = chunk_x;
        self.players[player_idx].last_chunk_z = chunk_z;
    }

    /// After an expensive operation or change in timings, it's important to
    /// call this function so our timings monitor doesn't think we're running
    /// behind.
    fn reset_timings(&mut self) {
        self.lag_time = Duration::ZERO;
        self.last_update_time = Instant::now();
        self.last_nspt = None;
        self.timings.reset_timings();
    }

    pub fn publish_world(&mut self) {
        if self.redpiler.is_active() {
            self.redpiler.flush(&mut self.world);
        }
        self.world.flush_block_changes();
        self.last_world_send_time = Instant::now();
    }

    pub fn start_redpiler(&mut self, options: CompilerOptions) {
        debug!("Starting redpiler");
        self.scoreboard
            .set_redpiler_state(&self.players, RedpilerState::Compiling);
        self.scoreboard
            .set_redpiler_options(&self.players, &options);

        let bounds = self.world.get_corners();
        // TODO: use monitor
        let monitor = Default::default();
        let ticks = mem::take(&mut self.world.to_be_ticked);

        let mut players_need_updates = HashSet::new();
        thread::scope(|s| {
            let handle = s.spawn(|| {
                self.redpiler
                    .compile(&self.world, bounds, options, ticks, monitor)
            });
            while !handle.is_finished() {
                // We'll update the players so that they don't time out.
                for player_idx in 0..self.players.len() {
                    if self.players[player_idx].update() {
                        // Unforunately we can't update a players view position
                        // since we don't have access to the world, but we can
                        // save the players that need updating for later.
                        players_need_updates.insert(player_idx);
                    }
                }
                thread::sleep(Duration::from_millis(20));
            }
        });

        // Now that we have ownership of the world again, we can update player view positions
        for player_idx in players_need_updates {
            self.update_view_pos_for_player(player_idx, false);
        }

        self.scoreboard
            .set_redpiler_state(&self.players, RedpilerState::Running);

        self.reset_timings();
    }

    /// Redpiler needs to reset implicitly in the case of any block changes done by a player. This
    /// can be
    pub fn reset_redpiler(&mut self) {
        if self.redpiler.is_active() {
            debug!("Discarding redpiler");
            let bounds = self.world.get_corners();
            self.redpiler.reset(&mut self.world, bounds);
            self.scoreboard
                .set_redpiler_state(&self.players, RedpilerState::Stopped);
            self.scoreboard
                .set_redpiler_options(&self.players, &Default::default());

            // reseting redpiler could cause a large amount of block updates
            self.reset_timings();
        }
    }

    pub fn send_message(&self, message: Message) {
        let _ = self.message_sender.send(message);
    }

    pub fn generate_timings_report(&self) -> Option<monitor::TimingsReport> {
        self.timings.generate_report()
    }

    pub fn tps(&self) -> Tps {
        self.tps
    }

    pub fn set_tps(&mut self, tps: Tps) {
        self.tps = tps;
        self.sleep_time = sleep_time_for_tps(tps);
        self.timings.set_tps(tps);
        self.reset_timings();
    }

    pub fn world_send_rate(&self) -> WorldSendRate {
        self.world.world_send_rate
    }

    pub fn set_world_send_rate(&mut self, rate: WorldSendRate) {
        self.world.world_send_rate = rate;
        self.world.pending_sounds.clear();
        self.last_world_send_time = Instant::now();
    }

    pub fn add_locked_player(&mut self, entity_id: EntityId) -> bool {
        self.locked_players.insert(entity_id)
    }

    pub fn remove_locked_player(&mut self, entity_id: EntityId) -> bool {
        self.locked_players.remove(&entity_id)
    }

    fn destroy_entity(&mut self, entity_id: u32) {
        let destroy_entity = CRemoveEntities {
            entity_ids: vec![entity_id as i32],
        }
        .encode();
        for player in &mut self.players {
            player.client.send_packet(&destroy_entity);
        }
    }

    pub fn leave_plot(&mut self, uuid: u128) -> Player {
        let player_idx = self.players.iter().position(|p| p.uuid == uuid).unwrap();
        let player = self.players.remove(player_idx);
        self.world.packet_senders.remove(&player.entity_id);

        let destroy_other_entities = CRemoveEntities {
            entity_ids: self.players.iter().map(|p| p.entity_id as i32).collect(),
        }
        .encode();
        player.client.send_packet(&destroy_other_entities);

        let chunk_offset_x = self.world.x << PLOT_SCALE;
        let chunk_offset_z = self.world.z << PLOT_SCALE;
        for chunk in &self.world.chunks {
            player.client.send_packet(
                &CUnloadChunk {
                    chunk_x: chunk_offset_x + chunk.x,
                    chunk_z: chunk_offset_z + chunk.z,
                }
                .encode(),
            );
        }
        self.destroy_entity(player.entity_id);
        self.locked_players.remove(&player.entity_id);
        self.scoreboard.remove_player(&player);
        player
    }

    fn chunk_in_plot_bounds(plot_x: i32, plot_z: i32, chunk_x: i32, chunk_z: i32) -> bool {
        let (x, z) = (chunk_x >> PLOT_SCALE, chunk_z >> PLOT_SCALE);
        plot_x == x && plot_z == z
    }

    pub fn in_plot_bounds(plot_x: i32, plot_z: i32, x: i32, z: i32) -> bool {
        Plot::chunk_in_plot_bounds(plot_x, plot_z, x >> 4, z >> 4)
    }

    pub fn get_center(plot_x: i32, plot_z: i32) -> (f64, f64) {
        const WIDTH: f64 = PLOT_BLOCK_WIDTH as f64;
        (
            plot_x as f64 * WIDTH + WIDTH / 2.0,
            plot_z as f64 * WIDTH + WIDTH / 2.0,
        )
    }

    fn handle_commands(&mut self) {
        let mut player_idx = 0;
        while player_idx < self.players.len() {
            let uuid = self.players[player_idx].uuid;
            let commands = mem::take(&mut self.players[player_idx].command_queue);
            for command_line in commands {
                info!(
                    "{} issued command: {}",
                    self.players[player_idx].username, command_line
                );
                let shared = Arc::clone(&self.shared);
                let result = shared.commands.execute(self, player_idx, &command_line);
                if let Err(error) = result {
                    error!("Failed to run command `{}`: {}", command_line, error);
                }
                if self.players.get(player_idx).map(|p| p.uuid) != Some(uuid) {
                    break;
                }
            }
            if self.players.get(player_idx).map(|p| p.uuid) == Some(uuid) {
                player_idx += 1;
            }
        }
    }

    fn handle_messages(&mut self) {
        while let Ok(message) = self.message_receiver.try_recv() {
            match message {
                BroadcastMessage::Chat(_sender, message) => {
                    for player in &mut self.players {
                        player.send_chat_message(&message);
                    }
                }
                BroadcastMessage::PlayerJoinedInfo(player_join_info) => {
                    let player_info = CPlayerInfoUpdate {
                        players: vec![CPlayerInfoUpdatePlayer {
                            uuid: player_join_info.uuid,
                            actions: CPlayerInfoActions {
                                add_player: Some(CPlayerInfoAddPlayer {
                                    name: player_join_info.username,
                                    properties: player_join_info.properties,
                                }),
                                update_gamemode: Some(player_join_info.gamemode.get_id()),
                                ..Default::default()
                            },
                        }],
                    }
                    .encode();
                    for player in &mut self.players {
                        player.client.send_packet(&player_info);
                    }
                }
                BroadcastMessage::PlayerLeft(uuid) => {
                    let player_info = CPlayerInfoRemove {
                        players: vec![uuid],
                    }
                    .encode();
                    for player in &mut self.players {
                        player.client.send_packet(&player_info);
                    }
                }
                BroadcastMessage::Shutdown => {
                    for player in &mut mem::take(&mut self.players) {
                        player.save();
                        player.kick("Server closed".into());
                    }
                    self.always_running = false;
                    self.running = false;
                    return;
                }
                BroadcastMessage::PlayerUpdateGamemode(uuid, gamemode) => {
                    let player_info = CPlayerInfoUpdate {
                        players: vec![CPlayerInfoUpdatePlayer {
                            uuid,
                            actions: CPlayerInfoActions {
                                update_gamemode: Some(gamemode.get_id()),
                                ..Default::default()
                            },
                        }],
                    }
                    .encode();
                    for player in &mut self.players {
                        player.client.send_packet(&player_info);
                    }
                }
            }
        }
        // Handle messages from the private message channel
        while let Ok(message) = self.priv_message_receiver.try_recv() {
            match message {
                PrivMessage::PlayerEnterPlot(player) => {
                    self.enter_plot(player);
                }
                PrivMessage::PlayerTeleportOther(mut player, username) => {
                    if let Some(other) = self.players.iter().find(|p| p.username == username) {
                        player.teleport(other.pos);
                    }
                    self.enter_plot(player);
                }
            }
        }
    }

    /// Remove players outside of the plot
    fn remove_oob_players(&mut self) {
        let mut outside_players = Vec::new();
        for player in 0..self.players.len() {
            let player = &mut self.players[player];
            if self.locked_players.contains(&player.entity_id) {
                continue;
            }
            let (plot_x, plot_z) = player.pos.plot_pos();
            if plot_x != self.world.x || plot_z != self.world.z {
                outside_players.push(player.uuid);
            }
        }

        for uuid in outside_players {
            let player = self.leave_plot(uuid);
            let player_leave_plot = Message::PlayerLeavePlot(player);
            self.message_sender.send(player_leave_plot).unwrap();
        }
    }

    /// Remove disconnected players
    fn remove_dc_players(&mut self) {
        let message_sender = &mut self.message_sender;

        let mut disconnected_players = Vec::new();
        self.players.retain(|player| {
            let alive = player.client.alive();
            if !alive {
                player.save();
                message_sender
                    .send(Message::PlayerLeft(player.uuid))
                    .unwrap();
                disconnected_players.push(player.entity_id);
            }
            alive
        });
        for entity_id in disconnected_players {
            self.world.packet_senders.remove(&entity_id);
            self.destroy_entity(entity_id);
        }
    }

    /// Update player view positions and handle packets
    fn update_players(&mut self) {
        for player_idx in 0..self.players.len() {
            if self.players[player_idx].update() {
                self.update_view_pos_for_player(player_idx, false);
            }
        }
        // Handle received packets
        for player_idx in 0..self.players.len() {
            self.handle_packets_for_player(player_idx);
        }
    }

    fn update(&mut self) {
        self.handle_messages();

        // Only tick if there are players in the plot
        if !self.players.is_empty() {
            self.timings.set_ticking(true);
            let now = Instant::now();
            self.last_player_time = now;

            let world_send_interval =
                Duration::try_from_secs_f64(1.0 / f64::from(self.world.world_send_rate.0))
                    .unwrap_or(Duration::MAX);

            let batch_interval = world_send_interval.min(Duration::from_millis(50));
            let max_batch_size = match self.last_nspt {
                Some(Duration::ZERO) | None => 1,
                Some(last_nspt) => {
                    let ticks_fit = batch_interval.as_nanos() / last_nspt.as_nanos();
                    // A tick previously took longer than the batch interval.
                    // Run at least one just so we're not stuck doing nothing
                    ticks_fit.clamp(1, u128::from(MAX_BATCH_SIZE)) as u32
                }
            };

            let batch_size = match self.tps {
                Tps::Limited(tps) if tps > 0.0 => {
                    // Very high rates can round the interval to zero and cause division by zero.
                    let dur_per_tick = Duration::try_from_secs_f64(1.0 / f64::from(tps))
                        .unwrap_or(Duration::MAX)
                        .max(Duration::from_nanos(1));
                    self.lag_time += now - self.last_update_time;
                    let ticks_behind = self.lag_time.as_nanos() / dur_per_tick.as_nanos();
                    self.lag_time = Duration::from_nanos(
                        (self.lag_time.as_nanos() % dur_per_tick.as_nanos()) as u64,
                    );
                    ticks_behind.min(u128::from(max_batch_size)) as u32
                }
                Tps::Unlimited => max_batch_size,
                _ => 0,
            };

            self.last_update_time = now;
            if batch_size != 0 {
                let mut ticks_completed = batch_size;
                if self.redpiler.is_active() {
                    self.tickn(batch_size as u64);
                    self.redpiler.flush(&mut self.world);
                } else {
                    for i in 0..batch_size {
                        self.tick();
                        if now.elapsed() > Duration::from_millis(200) {
                            ticks_completed = i + 1;
                            break;
                        }
                    }
                }
                self.last_nspt = Some(self.last_update_time.elapsed() / ticks_completed);
            }

            let now = Instant::now();
            let time_since_last_world_send = now - self.last_world_send_time;
            if time_since_last_world_send > world_send_interval {
                self.last_world_send_time = now;
                self.world.flush_block_changes();
            }
        } else {
            self.timings.set_ticking(false);
            // Unload plot after 600 seconds unless the plot should be always loaded
            if self.last_player_time.elapsed().as_secs() > 600 && !self.always_running {
                self.running = false;
                self.timings.stop();
            }
        }

        self.update_players();

        // Handle commands before removing players just in case they ran a command before leaving
        self.handle_commands();

        self.remove_dc_players();
        self.remove_oob_players();

        let autosave_interval =
            Duration::from_secs(self.shared.config.autosave_interval_seconds as u64);
        if self.shared.config.autosave_interval_seconds != 0
            && self.last_autosave_time.elapsed() >= autosave_interval
            && self.last_player_time >= self.last_autosave_time
            && !self.redpiler.is_active()
        {
            let start_time = Instant::now();
            self.save();
            self.last_autosave_time = Instant::now();
            debug!(
                "autosaving plot ({}, {}) took {:?}",
                self.world.x,
                self.world.z,
                start_time.elapsed()
            );
        }
    }

    fn run(&mut self, initial_player: Option<Player>) {
        if let Some(player) = initial_player {
            self.enter_plot(player);
        }

        while self.running {
            // Fast path, for super high RTPS
            if self.sleep_time <= Duration::from_millis(5) && !self.players.is_empty() {
                self.update();
                if self.tps != Tps::Unlimited {
                    thread::yield_now();
                }
                continue;
            }

            let before = Instant::now();
            self.update();
            let delta = Instant::now().duration_since(before);

            if delta < self.sleep_time {
                let sleep_time = self.sleep_time - delta;
                thread::sleep(sleep_time);
            } else {
                thread::yield_now();
            }
        }

        self.save();
    }

    /// This function is used in case of an error. It will try to send the player to spawn if this
    /// isn't already a spawn plot.
    fn send_player_away(plot_x: i32, plot_z: i32, player: &mut Player) {
        let (px, pz) = if plot_x == 0 && plot_z == 0 {
            // Can't send players to spawn if spawn crashed!
            Plot::get_center(1, 0)
        } else {
            Plot::get_center(0, 0)
        };
        player.teleport(PlayerPos::new(px, 64.0, pz));
    }

    pub fn load_and_run(
        x: i32,
        z: i32,
        channels: PlotChannels,
        always_running: bool,
        initial_player: Option<Player>,
        shared: Arc<Shared>,
    ) {
        thread::Builder::new()
            .name(format!("p{},{}", x, z))
            .spawn(
                move || match Plot::load(x, z, channels, always_running, shared) {
                    Ok(mut plot) => plot.run(initial_player),
                    Err((err, tx)) => {
                        if let Some(mut player) = initial_player {
                            player.send_error_message("There was an error loading that plot.");
                            Plot::send_player_away(x, z, &mut player);
                            tx.send(Message::PlayerLeavePlot(player)).unwrap();
                        }
                        tx.send(Message::PlotUnload(x, z)).unwrap();
                        panic!("{err:?}");
                    }
                },
            )
            .unwrap();
    }
}

impl Drop for Plot {
    fn drop(&mut self) {
        if !self.players.is_empty() {
            for player in &mut self.players {
                player.save(); // just in case

                let world = &self.world;
                Plot::send_player_away(world.x, world.z, player);

                player.send_error_message("The plot you were previously in has crashed!");
            }

            while !self.players.is_empty() {
                let uuid = self.players[0].uuid;
                let player = self.leave_plot(uuid);
                self.message_sender
                    .send(Message::PlayerLeavePlot(player))
                    .unwrap();
            }
        }
        self.reset_redpiler();
        self.world
            .chunks
            .iter_mut()
            .for_each(|chunk| chunk.compress());
        self.save();
        let world = &self.world;
        self.message_sender
            .send(Message::PlotUnload(world.x, world.z))
            .unwrap();
    }
}
