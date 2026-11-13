pub mod advance;
mod block_interaction;
#[cfg(feature = "claims")]
pub mod claims;
mod compile;
pub mod data;
mod packet_handlers;
mod scoreboard;
mod send_schedule;
pub mod simulation;
mod sound;
mod tick_history;
#[cfg(feature = "whitelist")]
mod whitelist;
pub mod world;

use crate::player::{EntityId, Gamemode, PacketSender, Player, PlayerPos};
use crate::server::{BroadcastMessage, Message, PlotFailure, PrivMessage};
use crate::shared::Shared;
use advance::Advances;
use bus::BusReader;
use mchprs_network::chunk::{encode_chunk, encode_empty_chunk};
use mchprs_network::packets::clientbound::*;
use mchprs_network::PlayerPacketSender;
use mchprs_redstone::{plate, Aabb};
use mchprs_save_data::plot_data::{Tps, WorldSendRate};
use scoreboard::{PlotStatus, Scoreboard};
use send_schedule::SendSchedule;
use simulation::PlotSimulation;
use std::collections::HashSet;
use std::mem;
use std::ops::ControlFlow;
use std::panic::{self, AssertUnwindSafe};
use std::sync::mpsc::{Receiver, Sender};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};
use tick_history::{TickHistory, TickRateReport};
use tracing::{error, info};
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

pub const ERROR_REDPILER_ACTIVE: &str =
    "This plot cannot be changed while redpiler is active. To stop it, run /redpiler reset.";

const SCOREBOARD_INTERVAL: Duration = Duration::from_millis(500);
const UPDATE_INTERVAL: Duration = Duration::from_millis(50);
const IDLE_TIME: Duration = Duration::from_secs(600);

pub struct Plot {
    pub world: PlotWorld,
    pub players: Vec<Player>,
    pub simulation: PlotSimulation,
    pub shared: Arc<Shared>,
    advances: Advances,
    ticks: u64,

    // Thread communication
    message_receiver: BusReader<BroadcastMessage>,
    message_sender: Sender<Message>,

    locked_players: HashSet<EntityId>,

    // Timings
    tps: Tps,
    send_schedule: SendSchedule,
    tick_history: TickHistory,
    /// The last time a player was in this plot
    last_player_time: Instant,
    last_scoreboard_time: Instant,
    reported_idle: bool,

    scoreboard: Scoreboard,
}

impl Plot {
    pub fn change_player_gamemode(&mut self, player_idx: usize, gamemode: Gamemode) {
        self.players[player_idx].set_gamemode(gamemode);
        let _ = self.message_sender.send(Message::PlayerUpdateGamemode(
            self.players[player_idx].uuid,
            gamemode,
        ));
    }

    pub fn teleport_player(&mut self, player_idx: usize, pos: PlayerPos) {
        self.players[player_idx].teleport(pos);
        self.on_players_moved();
    }

    fn on_players_moved(&mut self) {
        self.update_entity_boxes();
        if self.simulation.is_editable() && self.world.press_touched_plates() {
            self.publish_world();
        }
    }

    fn update_entity_boxes(&mut self) {
        let boxes: Vec<Aabb> = self
            .players
            .iter()
            .filter_map(|player| player.collision_box(player.pos))
            .collect();
        self.set_entity_boxes(boxes);
    }

    fn recheck_pressure_plates(&mut self) {
        let region = self.world.region();
        plate::recheck_pressed_plates(&mut self.world, region);
    }

    fn enter_plot(&mut self, player: Player) {
        self.reported_idle = false;
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
        self.refresh_scoreboard();
        self.scoreboard.add_player(&player);
        self.players.push(player);
        if self.players.len() == 1 {
            self.update_tick_rate();
        }
        self.update_view_pos_for_player(self.players.len() - 1, true);
        self.on_players_moved();
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
                if let Some(chunk) = self.world.chunk(chunk_x, chunk_z) {
                    let chunk_data = encode_chunk(chunk);
                    self.players[player_idx].client.send_packet(&chunk_data);
                }
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

    pub fn publish_world(&mut self) {
        self.show_world();
        self.send_schedule.sent(Instant::now());
    }

    fn show_world(&mut self) {
        self.world.flush_block_changes();
        self.send_sounds();
    }

    fn broadcast(&self, message: &str) {
        for player in &self.players {
            player.send_system_message(message);
        }
    }

    fn next_update(&self) -> Instant {
        let now = Instant::now();
        let send = self
            .send_schedule
            .next_send()
            .filter(|_| !self.players.is_empty());
        [self.simulation.next_tick(), send]
            .into_iter()
            .flatten()
            .fold(now + UPDATE_INTERVAL, Instant::min)
    }

    fn plot_status(&self) -> PlotStatus {
        let (redpiler, behind) = self.simulation.status();
        let compiling = self.simulation.is_compiling();
        PlotStatus {
            redpiler,
            advance: self.advances.progress(self.ticks),
            tps: self
                .tick_history
                .recent_rate(Instant::now())
                .filter(|_| !compiling),
            tps_target: self.tps,
            world_send_rate: self.world.world_send_rate,
            running_behind: behind,
        }
    }

    fn refresh_scoreboard(&mut self) {
        let status = self.plot_status();
        self.scoreboard.show(&self.players, &status);
        self.last_scoreboard_time = Instant::now();
    }

    pub fn send_message(&self, message: Message) {
        let _ = self.message_sender.send(message);
    }

    pub fn tick_rate_report(&self) -> Option<TickRateReport> {
        self.tick_history.report(Instant::now())
    }

    pub fn tps(&self) -> Tps {
        self.tps
    }

    pub fn set_tps(&mut self, tps: Tps) {
        self.tps = tps;
        self.world.mark_unsaved();
        self.update_tick_rate();
    }

    pub fn world_send_rate(&self) -> WorldSendRate {
        self.world.world_send_rate
    }

    pub fn set_world_send_rate(&mut self, rate: WorldSendRate) {
        self.world.world_send_rate = rate;
        self.world.mark_unsaved();
        self.world.pending_sounds.clear();
        self.send_schedule.set_rate(rate, Instant::now());
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
        self.update_entity_boxes();
        self.pause_if_empty();
        player
    }

    fn pause_if_empty(&mut self) {
        if self.players.is_empty() {
            self.stop_advancing();
            self.update_tick_rate();
        }
    }

    fn chunk_in_plot_bounds(plot_x: i32, plot_z: i32, chunk_x: i32, chunk_z: i32) -> bool {
        let (x, z) = (chunk_x >> PLOT_SCALE, chunk_z >> PLOT_SCALE);
        plot_x == x && plot_z == z
    }

    fn in_plot_bounds(plot_x: i32, plot_z: i32, x: i32, z: i32) -> bool {
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

    fn handle_messages(&mut self, private: &Receiver<PrivMessage>) -> ControlFlow<()> {
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
                    self.update_entity_boxes();
                    self.stop_redpiler(false);
                    return ControlFlow::Break(());
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
        while let Ok(message) = private.try_recv() {
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
                PrivMessage::Unload => {
                    if self.unload() {
                        return ControlFlow::Break(());
                    }
                }
            }
        }
        ControlFlow::Continue(())
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
        for &entity_id in &disconnected_players {
            self.world.packet_senders.remove(&entity_id);
            self.destroy_entity(entity_id);
        }
        if !disconnected_players.is_empty() {
            self.update_entity_boxes();
            self.pause_if_empty();
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

    fn update(&mut self, private: &Receiver<PrivMessage>) -> ControlFlow<()> {
        self.handle_messages(private)?;
        self.update_simulation();

        if !self.players.is_empty() {
            self.last_player_time = Instant::now();
            self.sample_ticks(Instant::now());
            if self.send_schedule.is_due(Instant::now()) {
                self.send_world();
            }

            if self.last_scoreboard_time.elapsed() >= SCOREBOARD_INTERVAL {
                self.refresh_scoreboard();
            }
        } else {
            self.tick_history.restart(Instant::now());
            self.report_idle();
        }
        self.autosave_if_due();

        self.update_players();

        // Handle commands before removing players just in case they ran a command before leaving
        self.handle_commands();

        self.remove_dc_players();
        self.remove_oob_players();
        ControlFlow::Continue(())
    }

    fn sample_ticks(&mut self, now: Instant) {
        if self.simulation.is_compiling() {
            self.tick_history.restart(now);
        } else if self.tick_history.is_sample_due(now) {
            self.tick_history.sample(now);
        }
    }

    fn is_idle(&self) -> bool {
        self.players.is_empty()
            && !self.advances.is_running()
            && self.last_player_time.elapsed() >= IDLE_TIME
    }

    fn report_idle(&mut self) {
        if self.reported_idle || !self.is_idle() || self.world.save_state.is_failing() {
            return;
        }
        self.reported_idle = true;
        let _ = self
            .message_sender
            .send(Message::PlotIdle(self.world.x, self.world.z));
    }

    fn unload(&mut self) -> bool {
        if self.is_idle() {
            self.stop_redpiler(false);
            self.save_for_unload();
            if self.world.save_state.is_saved() {
                return true;
            }
        }
        self.reported_idle = false;
        let _ = self
            .message_sender
            .send(Message::UnloadRefused(self.world.x, self.world.z));
        false
    }

    fn run(
        &mut self,
        initial_player: Option<Player>,
        private: &Receiver<PrivMessage>,
    ) -> Vec<Player> {
        if let Some(player) = initial_player {
            self.enter_plot(player);
        }

        while self.update(private).is_continue() {
            match self.next_update().checked_duration_since(Instant::now()) {
                Some(wait) if !wait.is_zero() => thread::sleep(wait),
                _ => thread::yield_now(),
            }
        }

        let mut players = Vec::new();
        self.evict_players(&mut players);
        self.save_for_unload();
        players
    }

    fn evict_players(&mut self, evicted: &mut Vec<Player>) {
        while let Some(player) = self.players.first() {
            let uuid = player.uuid;
            evicted.push(self.leave_plot(uuid));
        }
    }

    /// This function is used in case of an error. It will try to send the player to spawn if this
    /// isn't already a spawn plot.
    pub(crate) fn send_player_away(plot_x: i32, plot_z: i32, player: &mut Player) {
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
        initial_player: Option<Player>,
        shared: Arc<Shared>,
    ) {
        thread::Builder::new()
            .name(format!("p{},{}", x, z))
            .spawn(move || {
                let PlotChannels {
                    broadcasts,
                    server,
                    private,
                } = channels;
                let plot_sender = server.clone();
                let mut players = Vec::new();
                let mut saved = false;
                let exit = panic::catch_unwind(AssertUnwindSafe(|| {
                    let load = || {
                        Plot::load(x, z, broadcasts, plot_sender, shared)
                            .inspect_err(|error| error!("{error:?}"))
                    };
                    let Ok(Ok(mut plot)) = panic::catch_unwind(AssertUnwindSafe(load)) else {
                        players.extend(initial_player.map(|mut player| {
                            player.send_error_message(PlotFailure::Load.message());
                            Plot::send_player_away(x, z, &mut player);
                            player
                        }));
                        return Some(PlotFailure::Load);
                    };
                    let failure = plot.run_until_exit(initial_player, &private, &mut players);
                    saved = plot.world.save_state.is_saved();
                    failure
                }));
                let _ = server.send(Message::PlotExited {
                    x,
                    z,
                    players,
                    private,
                    failure: exit.unwrap_or(Some(PlotFailure::Crash)),
                    saved,
                });
            })
            .unwrap();
    }

    fn run_until_exit(
        &mut self,
        initial_player: Option<Player>,
        private: &Receiver<PrivMessage>,
        departed: &mut Vec<Player>,
    ) -> Option<PlotFailure> {
        let run = panic::catch_unwind(AssertUnwindSafe(|| self.run(initial_player, private)));
        match run {
            Ok(players) => {
                departed.extend(players);
                None
            }
            Err(_) => {
                let evacuate = || self.evacuate_after_crash(departed);
                let _ = panic::catch_unwind(AssertUnwindSafe(evacuate));
                departed.append(&mut self.players);
                Some(PlotFailure::Crash)
            }
        }
    }

    fn evacuate_after_crash(&mut self, evacuated: &mut Vec<Player>) {
        let (x, z) = (self.world.x, self.world.z);
        for player in &mut self.players {
            player.save();
            Plot::send_player_away(x, z, player);
            player.send_error_message(PlotFailure::Crash.message());
        }
        self.evict_players(evacuated);
    }
}

pub struct PlotChannels {
    pub broadcasts: BusReader<BroadcastMessage>,
    pub server: Sender<Message>,
    pub private: Receiver<PrivMessage>,
}
