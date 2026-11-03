mod advance;
#[cfg(feature = "claims")]
pub mod claims;
pub mod data;
mod packet_handlers;
pub mod redpiler;
mod scoreboard;
mod send_schedule;
mod tick_history;

use self::advance::Advance;
use self::redpiler::{tick_rate, Redpiler, UPDATE};
use self::scoreboard::Scoreboard;
use self::send_schedule::SendSchedule;
use self::tick_history::{TickHistory, TickRateReport};
use crate::interaction;
use crate::interaction::UseOnBlockContext;
use crate::player::{EntityId, Gamemode, PacketSender, Player, PlayerPos};
#[cfg(feature = "whitelist")]
use crate::profile::PlayerProfile;
use crate::server::{BroadcastMessage, Message, PrivMessage};
use crate::shared::Shared;
use anyhow::Error;
use bus::BusReader;
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::blocks::{Block, Instrument};
use mchprs_blocks::items::Item;
use mchprs_blocks::region::Region;
use mchprs_blocks::{BlockFace, BlockPos};
use mchprs_network::chunk::{encode_block_changes, encode_chunk, encode_empty_chunk};
use mchprs_network::packets::clientbound::*;
use mchprs_network::packets::serverbound::SUseItemOn;
use mchprs_network::PlayerPacketSender;
use mchprs_redpiler::backend::CompilePlan;
use mchprs_redpiler::compile::BuildContext;
use mchprs_redpiler::engine::{DriveError, EngineError, Input};
use mchprs_redpiler::simulation::Simulation;
use mchprs_redpiler_backends::BackendSetup;
use mchprs_redstone::pace::Pacer;
use mchprs_redstone::tick_queue::{TickEntry, TickPriority, TickQueue};
use mchprs_redstone::RedstoneWorld;
use mchprs_save_data::plot_data::{ChunkData, PlotData, SavedTick, Tps, WorldSendRate};
use mchprs_world::occupied_regions;
use mchprs_world::storage::Chunk;
use rustc_hash::FxHashMap;
use scoreboard::RedpilerState;
#[cfg(feature = "claims")]
use std::cmp::Ordering;
use std::collections::{HashMap, HashSet};
use std::mem;
use std::path::Path;
use std::sync::mpsc::{Receiver, Sender};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};
use tracing::{debug, error, info, warn};

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

const ERROR_IO_ONLY: &str = "This plot cannot be interacted with while redpiler only updates inputs and outputs. To stop redpiler, run `/redpiler reset`.";

const UPDATE_INTERVAL: Duration = Duration::from_millis(50);

pub struct Plot {
    pub world: PlotWorld,
    pub players: Vec<Player>,
    pub redpiler: Option<Redpiler>,
    pub shared: Arc<Shared>,
    advance: Option<Advance>,

    // Thread communication
    message_receiver: BusReader<BroadcastMessage>,
    message_sender: Sender<Message>,
    priv_message_receiver: Receiver<PrivMessage>,

    locked_players: HashSet<EntityId>,

    // Timings
    tps: Tps,
    origin: Instant,
    pacer: Pacer,
    send_schedule: SendSchedule,
    tick_history: TickHistory,
    /// The last time a player was in this plot
    last_player_time: Instant,
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

pub struct PlotWorld {
    pub x: i32,
    pub z: i32,
    pub chunks: Vec<Chunk>,
    pub ticks: TickQueue,
    pub packet_senders: HashMap<EntityId, PlayerPacketSender>,
    world_send_rate: WorldSendRate,
    pending_block_entities: FxHashMap<BlockPos, CBlockEntityData>,
    pending_sounds: FxHashMap<BlockPos, CSoundEffect>,
}

impl PlotWorld {
    fn get_chunk_index_for_chunk(&self, chunk_x: i32, chunk_z: i32) -> Option<usize> {
        let local_x = chunk_x - self.x * PLOT_WIDTH;
        let local_z = chunk_z - self.z * PLOT_WIDTH;
        if !(0..PLOT_WIDTH).contains(&local_x) || !(0..PLOT_WIDTH).contains(&local_z) {
            return None;
        }
        Some((local_x * PLOT_WIDTH + local_z) as usize)
    }

    fn get_chunk_index_for_block(&self, block_x: i32, block_z: i32) -> Option<usize> {
        let chunk_x = (block_x - (self.x * PLOT_BLOCK_WIDTH)) >> 4;
        let chunk_z = (block_z - (self.z * PLOT_BLOCK_WIDTH)) >> 4;
        if !(0..PLOT_WIDTH).contains(&chunk_x) || !(0..PLOT_WIDTH).contains(&chunk_z) {
            return None;
        }
        Some(((chunk_x << PLOT_SCALE) + chunk_z).unsigned_abs() as usize)
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
        for (_, sound) in self.pending_sounds.drain() {
            let encoded = sound.encode();
            for packet_sender in self.packet_senders.values() {
                packet_sender.send_packet(&encoded);
            }
        }
    }
}

impl BlockView for PlotWorld {
    fn block(&self, pos: BlockPos) -> Block {
        let Some(chunk_index) = self.get_chunk_index_for_block(pos.x, pos.z) else {
            return Block::Air;
        };
        let chunk = &self.chunks[chunk_index];
        Block::from_id(chunk.get_block((pos.x & 0xF) as u32, pos.y as u32, (pos.z & 0xF) as u32))
    }

    fn block_entity(&self, pos: BlockPos) -> Option<&BlockEntity> {
        let chunk_index = self.get_chunk_index_for_block(pos.x, pos.z)?;
        let chunk = &self.chunks[chunk_index];
        chunk.get_block_entity(BlockPos::new(pos.x & 0xF, pos.y, pos.z & 0xF))
    }

    fn occupied_regions(&self, region: Region) -> Vec<Region> {
        occupied_regions(region, |x, z| self.chunk(x, z))
    }
}

impl BlockStore for PlotWorld {
    /// Sets a block in storage. Returns true if a block was changed.
    fn set_block(&mut self, pos: BlockPos, block: Block) -> bool {
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
            block.get_id(),
        )
    }

    fn set_block_entity(&mut self, pos: BlockPos, block_entity: Option<BlockEntity>) {
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
            if had_sign && (block.is_sign() || block.is_wall_sign()) {
                let blank = BlockEntity::Sign(Box::default());
                if let Some(nbt) = blank.to_nbt(true) {
                    let block_entity_data = CBlockEntityData {
                        x: pos.x,
                        y: pos.y,
                        z: pos.z,
                        ty: blank.ty(),
                        nbt,
                    };
                    self.pending_block_entities.insert(pos, block_entity_data);
                }
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
}

impl RedstoneWorld for PlotWorld {
    fn schedule_tick(&mut self, pos: BlockPos, delay: u32, priority: TickPriority) {
        let block = self.block(pos);
        self.ticks.schedule(pos, delay, priority, block);
    }

    fn pending_tick_at(&self, pos: BlockPos) -> bool {
        self.ticks.is_pending(pos, self.block(pos))
    }

    fn play_note(&mut self, pos: BlockPos, instrument: Instrument, note: u8) {
        const RECORDS_SOUND_CATEGORY: i32 = 2;
        const NOTE_VOLUME: f32 = 3.0;
        // The note is mapped to [0, 31] to avoid ultra high pitches in case of invalid values.
        // The range [0, 31] is used even though it is different from the noteblock's note range
        // of [0, 24] because mapping to [0, 31] can be done efficiently using bitwise AND.
        let pitch = f32::exp2(((note % 32) as f32 - 12.0) / 12.0);
        self.play_sound(
            pos,
            instrument.to_sound_id(),
            RECORDS_SOUND_CATEGORY,
            NOTE_VOLUME,
            pitch,
        );
    }
}

impl Plot {
    fn tick(&mut self) {
        self.tick_history.record(1);
        if self.redpiler.is_some() {
            self.with_simulation(|simulation| simulation.run(1));
            return;
        }

        mchprs_redstone::tick_queue::run_scheduled_ticks(&mut self.world, |world| &mut world.ticks);
    }

    fn run_world_ticks(&mut self) {
        let Some(due) = self.pacer.due(self.now()) else {
            return;
        };
        let start = Instant::now();
        let batch_time = self.batch_time();
        let mut ran = 0;
        let mut settled = false;
        while ran < due.limit && (ran == 0 || start.elapsed() < batch_time) {
            if due.until_settled && self.is_settled() {
                settled = true;
                break;
            }
            self.tick();
            ran += 1;
        }
        if let Some(advanced) = self.pacer.ran(ran, settled, self.now()) {
            self.publish_world();
            self.record_advance(advanced);
        }
    }

    fn is_settled(&mut self) -> bool {
        match self.with_simulation(|simulation| simulation.settle(0)) {
            Some(settled) => settled.is_some(),
            None => self.world.ticks.entries().is_empty(),
        }
    }

    fn sample_ticks(&mut self, now: Instant) {
        if self.tick_history.is_sample_due(now) {
            self.tick_history.sample(now);
        }
    }

    pub fn send_block_corrections(&mut self, positions: &[BlockPos]) {
        self.publish_world();
        for pos in positions {
            let block_change = CBlockUpdate {
                block_id: self.world.block(*pos).get_id() as i32,
                x: pos.x,
                y: pos.y,
                z: pos.z,
            }
            .encode();
            for player in &self.players {
                player.client.send_packet(&block_change);
            }
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
        let from = self.players[player_idx].standing_on();
        self.players[player_idx].teleport(pos);
        self.step(from, self.players[player_idx].standing_on());
    }

    fn step(&mut self, from: Option<BlockPos>, to: Option<BlockPos>) {
        if from == to {
            if let Some(to) = to
                && self.redpiler.is_none()
                && let Some(false) = self.world.block(to).get_pressure_plate_powered()
            {
                self.set_pressure_plate(to, true);
            }
            return;
        }
        if let Some(from) = from
            && !self.are_players_on_block(from)
        {
            self.set_pressure_plate(from, false);
        }
        if let Some(to) = to {
            self.set_pressure_plate(to, true);
        }
    }

    fn set_pressure_plate(&mut self, pos: BlockPos, pressed: bool) {
        if self.world.block(pos).get_pressure_plate_powered().is_none() {
            return;
        }
        if self.redpiler.is_some() {
            self.drive_redpiler(pos, Input::Set(pressed.into()));
        } else {
            mchprs_redstone::set_pressure_plate(&mut self.world, pos, pressed);
        }
        self.publish_world();
    }

    fn are_players_on_block(&self, pos: BlockPos) -> bool {
        self.players
            .iter()
            .any(|player| player.standing_on() == Some(pos))
    }

    fn reset_pressure_plates(&mut self) {
        let region = self.world.region();
        for pos in self
            .world
            .occupied_regions(region)
            .into_iter()
            .flat_map(Region::positions)
        {
            if let Some(&mut powered) = self.world.block(pos).get_pressure_plate_powered()
                && powered != self.are_players_on_block(pos)
            {
                mchprs_redstone::set_pressure_plate(&mut self.world, pos, !powered);
            }
        }
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
        let to = player.standing_on();
        self.players.push(player);
        self.update_view_pos_for_player(self.players.len() - 1, true);
        self.step(None, to);
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

    fn handle_use_item_impl(&mut self, use_item_on: &SUseItemOn, player: usize) {
        let block_pos = BlockPos::new(use_item_on.x, use_item_on.y, use_item_on.z);
        let block_face = BlockFace::from_id(use_item_on.face as u32);

        let cancel = |plot: &mut Plot| {
            plot.send_block_corrections(&[block_pos, block_pos.offset(block_face)]);
        };

        let selected_slot = self.players[player].selected_slot as usize;
        let item_in_hand = if use_item_on.hand == 0 {
            // Slot in hotbar
            self.players[player].inventory[selected_slot + 36].clone()
        } else {
            // Slot for left hand
            self.players[player].inventory[45].clone()
        };

        if !Plot::in_plot_bounds(self.world.x, self.world.z, block_pos.x, block_pos.z) {
            self.players[player].send_system_message("Can't interact with blocks outside of plot");
            cancel(self);
            return;
        }

        if let Some(item) = &item_in_hand
            && item.item_type == Item::WoodenAxe
        {
            let same = self.players[player].worldedit_second_pos() == Some(block_pos);
            if !same {
                self.players[player].worldedit_set_second_pos(block_pos);
            }
            cancel(self);
            // FIXME: Because the client sends another packet after this for the left hand for
            // most blocks, redpiler will get reset anyways.
            return;
        }

        if let Some(redpiler) = &mut self.redpiler {
            let input = match self.world.block(block_pos) {
                Block::Lever { .. } => Some(Input::Toggle),
                Block::StoneButton { .. } => Some(Input::Press),
                _ => None,
            };
            let circuit = redpiler.simulation.circuit();
            let is_input = circuit
                .node_at(block_pos)
                .is_some_and(|node| circuit.node(node).is_input());
            if let Some(input) = input
                && is_input
                && !self.players[player].crouching
            {
                self.drive_redpiler(block_pos, input);
                self.publish_world();
                return;
            } else if redpiler.ports_only {
                self.players[player].send_error_message(ERROR_IO_ONLY);
                cancel(self);
                return;
            }
            self.reset_redpiler();
        }

        if let Some(item) = item_in_hand {
            let cancelled = interaction::use_item_on_block(
                &item,
                &mut self.world,
                UseOnBlockContext {
                    block_face,
                    block_pos,
                    player: &mut self.players[player],
                    cursor_y: use_item_on.cursor_y,
                    block_in_hitbox: self.shared.config.block_in_hitbox,
                },
            );
            if cancelled {
                cancel(self);
            }
            self.world.flush_block_changes();
            return;
        }

        let block = self.world.block(block_pos);
        if !self.players[player].crouching {
            interaction::on_use(
                block,
                &mut self.world,
                &mut self.players[player],
                block_pos,
                None,
            );
            self.world.flush_block_changes();
        }
    }

    fn handle_player_digging(&mut self, block_pos: BlockPos, player: usize) {
        let block = self.world.block(block_pos);

        if !Plot::in_plot_bounds(self.world.x, self.world.z, block_pos.x, block_pos.z) {
            self.players[player].send_system_message("Can't break blocks outside of plot");
            return;
        }

        // This worldedit wand stuff should probably be done in another file. It's good enough for
        // now.
        let item_in_hand = self.players[player].inventory
            [self.players[player].selected_slot as usize + 36]
            .clone();
        if let Some(item) = item_in_hand
            && item.item_type == Item::WoodenAxe
        {
            self.send_block_corrections(&[block_pos]);
            if let Some(pos) = self.players[player].worldedit_first_pos()
                && pos == block_pos
            {
                return;
            }
            self.players[player].worldedit_set_first_pos(block_pos);
            return;
        }

        if let Some(redpiler) = &self.redpiler
            && redpiler.ports_only
        {
            self.players[player].send_error_message(ERROR_IO_ONLY);
            self.send_block_corrections(&[block_pos]);
            return;
        }

        self.reset_redpiler();

        interaction::destroy(block, &mut self.world, block_pos);
        self.world.flush_block_changes();

        let effect = CWorldEvent {
            event: 2001,
            x: block_pos.x,
            y: block_pos.y,
            z: block_pos.z,
            data: block.get_id() as i32,
            disable_relative_volume: false,
        }
        .encode();
        for other_player in 0..self.players.len() {
            if player == other_player {
                continue;
            };
            self.players[other_player].client.send_packet(&effect);
        }
    }

    /// After an expensive operation or change in timings, it's important to
    /// call this function so we don't try to catch up on the ticks it held up.
    fn reset_timings(&mut self) {
        self.pacer.skip_overdue(self.now());
        self.tick_history.restart(Instant::now());
    }

    pub fn publish_world(&mut self) {
        if let Some(changes) = self.with_simulation(|simulation| simulation.sync()) {
            changes.apply(&mut self.world);
        }
        self.world.flush_block_changes();
        self.send_schedule.sent(self.now());
    }

    fn now(&self) -> Duration {
        self.origin.elapsed()
    }

    fn batch_time(&self) -> Duration {
        self.send_schedule
            .interval()
            .map_or(UPDATE_INTERVAL, |interval| interval.min(UPDATE_INTERVAL))
    }

    fn next_update(&self) -> Instant {
        let now = Instant::now();
        if self.advance.is_some() {
            return now;
        }
        if self.players.is_empty() {
            return now + UPDATE_INTERVAL;
        }
        [self.pacer.next_tick(), self.send_schedule.next_send()]
            .into_iter()
            .flatten()
            .map(|due| self.origin + due)
            .fold(now + UPDATE_INTERVAL, Instant::min)
    }

    /// Writes the state of every node to the world, including the nodes that aren't observed.
    pub fn sync_redpiler(&mut self) {
        if let Some(changes) = self.with_simulation(|simulation| simulation.sync_all()) {
            changes.apply(&mut self.world);
        }
    }

    /// Stops redpiler when the engine fails, since its state is lost.
    fn with_simulation<T>(
        &mut self,
        operation: impl FnOnce(&mut Simulation) -> Result<T, EngineError>,
    ) -> Option<T> {
        let redpiler = self.redpiler.as_mut()?;
        match operation(&mut redpiler.simulation) {
            Ok(value) => Some(value),
            Err(error) => {
                error!("Redpiler failed and was stopped: {error}");
                self.redpiler = None;
                self.scoreboard
                    .set_redpiler_state(&self.players, RedpilerState::Stopped);
                self.scoreboard.set_redpiler_setup(&self.players, None);
                None
            }
        }
    }

    /// An input the circuit rejects is ignored; an engine failure stops redpiler.
    fn drive_redpiler(&mut self, pos: BlockPos, input: Input) {
        self.with_simulation(|simulation| match simulation.drive(pos, input) {
            Ok(()) => Ok(()),
            Err(DriveError::Engine(error)) => Err(error),
            Err(error) => {
                warn!("Redpiler rejected {input:?} at {pos}: {error}");
                Ok(())
            }
        });
    }

    pub fn start_redpiler(
        &mut self,
        setup: BackendSetup,
        plan: CompilePlan,
    ) -> Result<(), EngineError> {
        debug!("Starting redpiler with {setup}");
        self.reset_pressure_plates();
        self.scoreboard
            .set_redpiler_state(&self.players, RedpilerState::Compiling);
        self.scoreboard
            .set_redpiler_setup(&self.players, Some(&setup));

        let region = self.world.region();
        let ticks = mem::take(&mut self.world.ticks).into_entries();
        let ports_only = plan.extract.ports_only;

        let mut players_need_updates = HashSet::new();
        let result = thread::scope(|s| {
            let handle = s.spawn(|| -> Result<Simulation, EngineError> {
                let start = Instant::now();
                let context = &mut BuildContext::silent();
                let circuit = mchprs_redpiler::compile::compile(
                    &self.world,
                    region,
                    &ticks,
                    &plan.extract,
                    &plan.passes,
                    context,
                )?;
                let engine = setup.backend().build(&circuit, setup.flags(), context)?;
                debug!("Compile completed in {:?}", start.elapsed());
                Ok(Simulation::new(Arc::new(circuit), engine))
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
            handle.join().expect("the compile thread panicked")
        });

        // Now that we have ownership of the world again, we can update player view positions
        for player_idx in players_need_updates {
            self.update_view_pos_for_player(player_idx, false);
        }

        let simulation = match result {
            Ok(simulation) => simulation,
            Err(error) => {
                self.world.ticks = TickQueue::new(ticks);
                self.scoreboard
                    .set_redpiler_state(&self.players, RedpilerState::Stopped);
                self.scoreboard.set_redpiler_setup(&self.players, None);
                return Err(error);
            }
        };
        self.redpiler = Some(Redpiler {
            setup,
            ports_only,
            simulation,
        });
        self.scoreboard
            .set_redpiler_state(&self.players, RedpilerState::Running);

        self.reset_timings();
        Ok(())
    }

    /// Redpiler needs to reset implicitly in the case of any block changes done by a player. This
    /// can be
    pub fn reset_redpiler(&mut self) {
        if self.redpiler.is_some() {
            debug!("Discarding redpiler");
            self.sync_redpiler();
            let ticks = self.with_simulation(|simulation| simulation.pending_ticks());
            let Some(redpiler) = self.redpiler.take() else {
                return;
            };
            for tick in ticks.into_iter().flatten() {
                self.world
                    .schedule_tick(tick.pos, tick.ticks_left, tick.priority);
            }
            if redpiler.setup.flags().is_on(&UPDATE) {
                let region = self.world.region();
                for pos in self
                    .world
                    .occupied_regions(region)
                    .into_iter()
                    .flat_map(Region::positions)
                {
                    let block = self.world.block(pos);
                    mchprs_redstone::update(block, &mut self.world, pos);
                }
            }
            self.scoreboard
                .set_redpiler_state(&self.players, RedpilerState::Stopped);
            self.scoreboard.set_redpiler_setup(&self.players, None);

            // reseting redpiler could cause a large amount of block updates
            self.reset_timings();
        }
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
        self.pacer.set_rate(tick_rate(tps), self.now());
        self.reset_timings();
    }

    pub fn world_send_rate(&self) -> WorldSendRate {
        self.world.world_send_rate
    }

    pub fn set_world_send_rate(&mut self, rate: WorldSendRate) {
        self.world.world_send_rate = rate;
        self.world.pending_sounds.clear();
        self.send_schedule.set_rate(rate, self.now());
    }

    pub fn add_locked_player(&mut self, entity_id: EntityId) -> bool {
        self.locked_players.insert(entity_id)
    }

    pub fn remove_locked_player(&mut self, entity_id: EntityId) -> bool {
        self.locked_players.remove(&entity_id)
    }

    #[cfg(feature = "whitelist")]
    pub fn whitelist_add(&self, username: String, packet_sender: PlayerPacketSender) {
        if !self.shared.config.whitelist {
            packet_sender.send_error_message("Whitelist is not enabled!");
            return;
        }
        packet_sender.send_system_message(&format!("Adding {} to whitelist...", username));
        self.look_up_profile(username, packet_sender, |profile, packet_sender| {
            Message::WhitelistAdd(profile.uuid.0, profile.username, packet_sender)
        });
    }

    #[cfg(feature = "whitelist")]
    pub fn whitelist_remove(&self, username: String, packet_sender: PlayerPacketSender) {
        if !self.shared.config.whitelist {
            packet_sender.send_error_message("Whitelist is not enabled!");
            return;
        }
        self.look_up_profile(username, packet_sender, |profile, packet_sender| {
            Message::WhitelistRemove(profile.uuid.0, packet_sender)
        });
    }

    #[cfg(feature = "whitelist")]
    fn look_up_profile(
        &self,
        username: String,
        packet_sender: PlayerPacketSender,
        message: fn(PlayerProfile, PlayerPacketSender) -> Message,
    ) {
        let sender = self.message_sender.clone();
        thread::spawn(move || match PlayerProfile::lookup_by_username(&username) {
            Ok(profile) => {
                let _ = sender.send(message(profile, packet_sender));
            }
            Err(error) => {
                warn!("Failed to look up profile for {username:?}: {error}");
                packet_sender.send_error_message(
                    "Unable to look up that player. Check the name and try again.",
                );
            }
        });
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
        self.step(player.standing_on(), None);
        player
    }

    fn chunk_in_plot_bounds(plot_x: i32, plot_z: i32, chunk_x: i32, chunk_z: i32) -> bool {
        let (x, z) = (chunk_x >> PLOT_SCALE, chunk_z >> PLOT_SCALE);
        plot_x == x && plot_z == z
    }

    pub fn in_plot_bounds(plot_x: i32, plot_z: i32, x: i32, z: i32) -> bool {
        Plot::chunk_in_plot_bounds(plot_x, plot_z, x >> 4, z >> 4)
    }

    #[cfg(feature = "claims")]
    pub fn claim_plot(&mut self, plot_x: i32, plot_z: i32, player_idx: usize) {
        let uuid = self.players[player_idx].uuid;
        self.shared
            .claims
            .claim_plot(plot_x, plot_z, &format!("{:032x}", uuid));
        let center = Plot::get_center(plot_x, plot_z);
        self.teleport_player(player_idx, PlayerPos::new(center.0, 64.0, center.1));
        self.players[player_idx]
            .send_system_message(&format!("Claimed plot {},{}", plot_x, plot_z));
    }

    pub fn get_center(plot_x: i32, plot_z: i32) -> (f64, f64) {
        const WIDTH: f64 = PLOT_BLOCK_WIDTH as f64;
        (
            plot_x as f64 * WIDTH + WIDTH / 2.0,
            plot_z as f64 * WIDTH + WIDTH / 2.0,
        )
    }

    #[cfg(feature = "claims")]
    pub fn get_next_plot(plot_x: i32, plot_z: i32) -> (i32, i32) {
        let x = plot_x.abs();
        let z = plot_z.abs();

        match x.cmp(&z) {
            Ordering::Greater => {
                if plot_x > 0 {
                    (plot_x, plot_z + 1)
                } else {
                    (plot_x, plot_z - 1)
                }
            }
            Ordering::Less => {
                if plot_z > 0 {
                    (plot_x - 1, plot_z)
                } else {
                    (plot_x + 1, plot_z)
                }
            }
            Ordering::Equal => {
                if plot_x == plot_z && plot_x > 0 || plot_x == x {
                    (plot_x, plot_z + 1)
                } else if plot_z == z {
                    (plot_x, plot_z - 1)
                } else {
                    (plot_x + 1, plot_z)
                }
            }
        }
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
                        self.step(player.standing_on(), None);
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
                disconnected_players.push((player.entity_id, player.standing_on()));
            }
            alive
        });
        for (entity_id, standing_on) in disconnected_players {
            self.world.packet_senders.remove(&entity_id);
            self.destroy_entity(entity_id);
            self.step(standing_on, None);
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
        self.continue_advancing();

        // Only tick if there are players in the plot
        if !self.players.is_empty() {
            self.last_player_time = Instant::now();

            self.run_world_ticks();

            self.sample_ticks(Instant::now());
            if self.send_schedule.is_due(self.now()) {
                self.publish_world();
            }
        } else {
            self.pacer.skip_overdue(self.now());
            self.tick_history.restart(Instant::now());
            if self.pacer.is_advancing() {
                self.run_world_ticks();
            }
            // Unload plot after 600 seconds unless the plot should be always loaded
            if self.last_player_time.elapsed().as_secs() > 600
                && !self.always_running
                && self.advance.is_none()
            {
                self.running = false;
            }
        }

        self.update_players();

        // Handle commands before removing players just in case they ran a command before leaving
        self.handle_commands();

        self.remove_dc_players();
        self.remove_oob_players();
    }

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
        let chunk_x_offset = x << PLOT_SCALE;
        let chunk_z_offset = z << PLOT_SCALE;
        let chunks: Vec<Chunk> = plot_data
            .chunk_data
            .into_iter()
            .enumerate()
            .map(|(i, c)| {
                c.load(
                    chunk_x_offset + i as i32 / PLOT_WIDTH,
                    chunk_z_offset + i as i32 % PLOT_WIDTH,
                )
            })
            .collect();
        if chunks.len() != NUM_CHUNKS {
            error!("This plot has the wrong number of chunks!");
            let possible_scale = (chunks.len() as f64).sqrt().log2();
            error!("Note: it most likely came from a server running plot scale {}, this server is running a plot scale of {}", possible_scale, PLOT_SCALE);
        }
        let world = PlotWorld {
            x,
            z,
            chunks,
            ticks: TickQueue::new(
                plot_data
                    .pending_ticks
                    .into_iter()
                    .map(|tick| {
                        TickEntry::new(
                            tick.pos,
                            tick.ticks_left,
                            tick.priority,
                            Block::from_id(tick.block_id),
                        )
                    })
                    .collect(),
            ),
            packet_senders: HashMap::new(),
            world_send_rate: plot_data.world_send_rate,
            pending_block_entities: FxHashMap::default(),
            pending_sounds: FxHashMap::default(),
        };
        let tps = plot_data.tps;
        let now = Instant::now();
        Plot {
            last_player_time: now,
            origin: now,
            pacer: Pacer::new(tick_rate(tps), Duration::ZERO),
            send_schedule: SendSchedule::new(plot_data.world_send_rate, Duration::ZERO),
            message_receiver: channels.broadcasts,
            message_sender: channels.server,
            priv_message_receiver: channels.private,
            players: Vec::new(),
            locked_players: HashSet::new(),
            running: true,
            tps,
            always_running,
            redpiler: None,
            advance: None,
            tick_history: TickHistory::new(now),
            scoreboard: Default::default(),
            shared,
            world,
        }
    }

    fn load(
        x: i32,
        z: i32,
        channels: PlotChannels,
        always_running: bool,
        shared: Arc<Shared>,
    ) -> Result<Plot, (Error, Sender<Message>)> {
        let plot_path = format!("./world/plots/p{},{}", x, z);
        let mut plot = if Path::new(&plot_path).exists() {
            match data::load_plot(plot_path) {
                Ok(data) => Plot::from_data(data, x, z, channels, always_running, shared),
                Err(err) => {
                    return Result::Err((
                        err.context(format!("error loading plot {},{}", x, z)),
                        channels.server,
                    ))
                }
            }
        } else {
            let data = shared.empty_plot.clone();
            Plot::from_data(data, x, z, channels, always_running, shared)
        };
        plot.reset_pressure_plates();
        Ok(plot)
    }

    fn save(&mut self) {
        let world = &mut self.world;
        let chunk_data: Vec<ChunkData<'_>> = world.chunks.iter_mut().map(ChunkData::new).collect();
        let data = PlotData {
            tps: self.tps,
            world_send_rate: world.world_send_rate,
            chunk_data,
            pending_ticks: world
                .ticks
                .entries()
                .iter()
                .map(|tick| SavedTick {
                    ticks_left: tick.ticks_left,
                    priority: tick.priority,
                    pos: tick.pos,
                    block_id: tick.block.get_id(),
                })
                .collect(),
        };
        data.save_to_file(format!("./world/plots/p{},{}", world.x, world.z))
            .unwrap();

        self.reset_timings();
    }

    fn run(&mut self, initial_player: Option<Player>) {
        if let Some(player) = initial_player {
            self.enter_plot(player);
        }

        while self.running {
            self.update();
            match self.next_update().checked_duration_since(Instant::now()) {
                Some(wait) if !wait.is_zero() => thread::sleep(wait),
                _ => thread::yield_now(),
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
