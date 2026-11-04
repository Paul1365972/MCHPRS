use crate::commands::command_registry;
use crate::commands::suggestions::{CommandSuggestions, SuggestionSource};
use crate::commands::value::PlayerTarget;
use crate::config::ServerConfig;
use crate::player::{Gamemode, PacketSender, Player};
#[cfg(feature = "claims")]
use crate::plot::claims::Claims;
use crate::plot::{data, Plot, PlotChannels, PLOT_BLOCK_HEIGHT};
use crate::shared::Shared;
use crate::utils;
#[cfg(feature = "whitelist")]
use crate::utils::HyphenatedUUID;
use bus::Bus;
use hmac::{Hmac, Mac};
use mchprs_blocks::MC_VERSION;
use mchprs_network::packets::clientbound::{
    CCommandSuggestionsResponse, CConfigurationPluginMessage, CDisconnectLogin,
    CFinishConfiguration, CGameEvent, CGameEventType, CLogin, CLoginPluginRequest, CLoginSuccess,
    CPlayerInfoActions, CPlayerInfoAddPlayer, CPlayerInfoUpdate, CPlayerInfoUpdatePlayer, CPong,
    CRegistryBiome, CRegistryBiomeEffects, CRegistryData, CRegistryDataCodec,
    CRegistryDimensionType, CResponse, CSetCompression, CSetContainerContent, CSetHeldItem,
    CSynchronizePlayerPosition, ClientBoundPacket, UpdateTime,
};
use mchprs_network::packets::serverbound::{
    SAcknowledgeFinishConfiguration, SHandshake, SLoginAcknowledged, SLoginPluginResponse,
    SLoginStart, SPing, SRequest, ServerBoundPacketHandler, VelocityResponseData,
};
use mchprs_network::packets::{PacketEncoderExt, PlayerProperty, SlotData, COMPRESSION_THRESHOLD};
use mchprs_network::text::TextComponent;
#[cfg(feature = "whitelist")]
use mchprs_network::PlayerPacketSender;
use mchprs_network::{NetworkServer, NetworkState, PROTOCOL_VERSION};
use rustc_hash::FxHashMap;
#[cfg(feature = "whitelist")]
use serde::{Deserialize, Serialize};
use serde_json::json;
use sha2::Sha256;
use std::collections::HashMap;
use std::fs;
use std::io::Cursor;
#[cfg(feature = "whitelist")]
use std::io::{self, Write as _};
#[cfg(not(feature = "claims"))]
use std::path::Path;
use std::sync::mpsc::{self, Receiver, Sender, SyncSender};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tracing::{error, info, warn};

/// `Message` gets send from a plot thread to the server thread.
#[derive(Debug)]
pub enum Message {
    /// This message is sent to the server thread when a player's command suggestions need the
    /// names of online players.
    CommandSuggestions(CommandSuggestions, SyncSender<CCommandSuggestionsResponse>),
    /// This message is sent to the server thread when a player sends a chat message,
    /// It contains the uuid and name of the player and the raw message the player sent.
    ChatInfo(u128, String, String),
    /// This message is sent to the server thread when a player joins the server.
    PlayerJoined(Player),
    /// This message is sent to the server thread when a player leaves the server.
    PlayerLeft(u128),
    /// This message is sent to the server thread when a player goes outside of their plot.
    PlayerLeavePlot(Player),
    /// This message is sent to the server thread when a player runs /tp <name>.
    PlayerTeleportOther(Player, PlayerTarget),
    /// This message is sent to the server thread when a player changes their gamemode.
    PlayerUpdateGamemode(u128, Gamemode),
    /// This message is sent to the server thread when a plot unloads itself.
    PlotUnload(i32, i32),
    /// This message is sent to the server thread when a player runs /whitelist add.
    #[cfg(feature = "whitelist")]
    WhitelistAdd(u128, String, PlayerPacketSender),
    /// This message is sent to the server thread when a player runs /whitelist remove.
    #[cfg(feature = "whitelist")]
    WhitelistRemove(u128, PlayerPacketSender),
    /// This message is sent to the server thread when a player runs /stop.
    Shutdown,
}

/// `BroadcastMessage` gets broadcasted from the server thread to all the plot threads.
/// This happens when there is a chat message, a player joins or leaves, or the server
/// shuts down.
#[derive(Debug, Clone)]
pub enum BroadcastMessage {
    /// This message is broadcasted for chat messages. It contains the uuid of the player and
    /// the raw json data to send to the clients.
    Chat(u128, Vec<TextComponent>),
    /// This message is broadcasted when a player joins the server. It is used to update
    /// the tab-list on all connected clients.
    PlayerJoinedInfo(PlayerJoinInfo),
    /// This message is broadcasted when a player leaves the server. It is used to update
    /// the tab-list on all connected clients.
    PlayerLeft(u128),
    /// This message is broadcasted when a player changes their gamemode,
    PlayerUpdateGamemode(u128, Gamemode),
    /// This message is broadcasted when the server is stopping, either through the stop
    /// command or through the ctrl+c handler.
    Shutdown,
}

/// `PrivMessage` gets send from the server thread directly to a plot thread.
/// This only happens when a player is getting transfered to a plot.
#[derive(Debug)]
pub enum PrivMessage {
    PlayerEnterPlot(Player),
    PlayerTeleportOther(Player, String),
}

/// This is the data that gets sent in the `PlayerJoinedInfo` broadcast message.
/// It contains imformation such as the player's username, uuid, and skin.
#[derive(Debug, Clone)]
pub struct PlayerJoinInfo {
    pub username: String,
    pub uuid: u128,
    pub gamemode: Gamemode,
    pub properties: Vec<PlayerProperty>,
}

#[derive(Debug, Clone)]
struct PlayerListEntry {
    plot_x: i32,
    plot_z: i32,
    username: String,
    properties: Vec<PlayerProperty>,
    gamemode: Gamemode,
}

struct PlotListEntry {
    plot_x: i32,
    plot_z: i32,
    priv_message_sender: mpsc::Sender<PrivMessage>,
}

#[cfg(feature = "whitelist")]
#[derive(Clone, Serialize, Deserialize)]
struct WhitelistEntry {
    uuid: HyphenatedUUID,
    name: String,
}

/// This represents a minecraft server
pub struct MinecraftServer {
    network: NetworkServer,
    broadcaster: Bus<BroadcastMessage>,
    receiver: Receiver<Message>,
    plot_sender: Sender<Message>,
    online_players: FxHashMap<u128, PlayerListEntry>,
    running_plots: Vec<PlotListEntry>,
    #[cfg(feature = "whitelist")]
    whitelist: Option<Vec<WhitelistEntry>>,
    shared: Arc<Shared>,
}

impl MinecraftServer {
    /// Start the server
    pub fn run() {
        std::panic::set_hook(Box::new(|panic_info| {
            let backtrace = std::backtrace::Backtrace::force_capture();
            let thread = std::thread::current();
            let name = thread.name().unwrap_or("unnamed");
            error!("Thread '{name}' {panic_info}\n{backtrace}");
        }));

        info!("Starting server...");
        let start_time = Instant::now();

        // Create world folders if they don't exist yet
        fs::create_dir_all("./world/players").unwrap();
        fs::create_dir_all("./world/plots").unwrap();
        fs::create_dir_all("./schems").unwrap();

        #[cfg(not(feature = "claims"))]
        if Path::new("./world/plots.db").exists() {
            panic!("world/plots.db holds plot claims, but this build has no claims feature. Build with `--features claims` to keep them, or delete the file");
        }
        let config = ServerConfig::load();
        let commands = command_registry(&config);
        let empty_plot = data::empty_plot();
        #[cfg(feature = "claims")]
        let claims = Claims::open("./world/plots.db").expect("Error opening plot database!");
        let shared = Arc::new(Shared {
            config,
            commands,
            empty_plot,
            #[cfg(feature = "claims")]
            claims,
        });

        // Create thread messaging structs
        let (plot_tx, server_rx) = mpsc::channel();
        let bus = Bus::new(100);
        let ctrl_handler_sender = plot_tx.clone();

        ctrlc::set_handler(move || {
            ctrl_handler_sender.send(Message::Shutdown).unwrap();
        })
        .expect("There was an error setting the ctrlc handler");

        #[cfg(not(feature = "whitelist"))]
        if shared.config.whitelist {
            panic!("Config.toml enables the whitelist, but this build has no whitelist feature. Build with `--features whitelist`, or set `whitelist = false`");
        }

        #[cfg(feature = "whitelist")]
        let whitelist = shared.config.whitelist.then(|| {
            let content = match fs::read("whitelist.json") {
                Ok(content) => content,
                Err(error) if error.kind() == io::ErrorKind::NotFound => {
                    fs::write("whitelist.json", b"[]").expect("Failed to create whitelist.json");
                    Vec::new()
                }
                Err(error) => panic!("Failed to read whitelist.json: {error}"),
            };
            if content.trim_ascii().is_empty() {
                Vec::new()
            } else {
                serde_json::from_slice(&content).expect("Invalid whitelist.json")
            }
        });

        // Create server struct
        let mut server = MinecraftServer {
            network: NetworkServer::new(shared.config.bind_address.clone()),
            broadcaster: bus,
            receiver: server_rx,
            plot_sender: plot_tx,
            online_players: FxHashMap::default(),
            running_plots: Vec::new(),
            #[cfg(feature = "whitelist")]
            whitelist,
            shared,
        };

        // Load the spawn area plot on server start
        // This plot should be always active
        server.spawn_plot(0, 0, true, None);

        info!("Done! Start took {:?}", start_time.elapsed());

        loop {
            server.update();
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    /// Updates the player's location on the `online_players` list
    fn update_player_entry(&mut self, uuid: u128, plot_x: i32, plot_z: i32) {
        let player = self.online_players.get_mut(&uuid);
        if let Some(player) = player {
            player.plot_x = plot_x;
            player.plot_z = plot_z;
        }
    }

    /// Removes the plot entry from the `running_plots` list
    fn handle_plot_unload(&mut self, plot_x: i32, plot_z: i32) {
        let index = self
            .running_plots
            .iter()
            .position(|p| p.plot_x == plot_x && p.plot_z == plot_z);
        if let Some(index) = index {
            self.running_plots.remove(index);
        }
    }

    fn graceful_shutdown(&mut self) {
        info!("Commencing graceful shutdown...");
        self.broadcaster.broadcast(BroadcastMessage::Shutdown);
        // Wait for all plots to save and unload
        while !self.running_plots.is_empty() {
            while let Ok(message) = self.receiver.try_recv() {
                if let Message::PlotUnload(plot_x, plot_z) = message {
                    self.handle_plot_unload(plot_x, plot_z);
                }
                std::thread::sleep(Duration::from_millis(2));
            }
        }

        std::process::exit(0);
    }

    fn spawn_plot(
        &mut self,
        plot_x: i32,
        plot_z: i32,
        always_running: bool,
        initial_player: Option<Player>,
    ) {
        let (priv_message_sender, private) = mpsc::channel();
        let channels = PlotChannels {
            broadcasts: self.broadcaster.add_rx(),
            server: self.plot_sender.clone(),
            private,
        };
        let shared = Arc::clone(&self.shared);
        Plot::load_and_run(
            plot_x,
            plot_z,
            channels,
            always_running,
            initial_player,
            shared,
        );
        self.running_plots.push(PlotListEntry {
            plot_x,
            plot_z,
            priv_message_sender,
        });
    }

    fn send_player_to_plot(&mut self, player: Player, new_entry: bool) {
        let (plot_x, plot_z) = player.pos.plot_pos();

        if new_entry {
            let player_list_entry = PlayerListEntry {
                plot_x,
                plot_z,
                username: player.username.clone(),
                properties: player.properties.clone(),
                gamemode: player.gamemode,
            };
            self.online_players.insert(player.uuid, player_list_entry);
        } else {
            self.update_player_entry(player.uuid, plot_x, plot_z);
        }

        let plot_loaded = self
            .running_plots
            .iter()
            .any(|p| p.plot_x == plot_x && p.plot_z == plot_z);
        if !plot_loaded {
            self.spawn_plot(plot_x, plot_z, false, Some(player));
        } else {
            let plot_list_entry = self
                .running_plots
                .iter()
                .find(|p| p.plot_x == plot_x && p.plot_z == plot_z)
                .unwrap();
            let _ = plot_list_entry
                .priv_message_sender
                .send(PrivMessage::PlayerEnterPlot(player));
        }
    }

    fn handle_player_enter_play(&mut self, client_idx: usize) {
        let client = self.network.handshaking_clients.remove(client_idx);

        let uuid = client.uuid.unwrap();
        let username = client.username.clone().unwrap();
        let properties = client.properties.clone();
        let player = Player::load_player(uuid, username, properties, client.into());

        let join_game = CLogin {
            entity_id: player.entity_id as i32,
            is_hardcore: false,
            dimension_names: vec!["minecraft:overworld".to_owned()],
            max_players: self.shared.config.max_players as i32,
            view_distance: self.shared.config.view_distance as i32,
            simulation_distance: self.shared.config.view_distance as i32,
            reduced_debug_info: false,
            enable_respawn_screen: false,
            do_limited_crafting: false,
            dimension_type: "minecraft:overworld".to_owned(),
            dimension_name: "minecraft:overworld".to_owned(),
            hashed_seed: 0,
            gamemode: player.gamemode.get_id() as u8,
            previous_gamemode: 1,
            is_debug: false,
            is_flat: true,
            death_location: None,
            portal_cooldown: 0,
        }
        .encode();
        player.client.send_packet(&join_game);

        let entity_metadata = player.metadata_packet().encode();
        player.client.send_packet(&entity_metadata);

        // Send the player's position and rotation.
        let player_pos_and_look = CSynchronizePlayerPosition {
            x: player.pos.x,
            y: player.pos.y,
            z: player.pos.z,
            yaw: player.yaw,
            pitch: player.pitch,
            flags: 0,
            teleport_id: 0,
        }
        .encode();
        player.client.send_packet(&player_pos_and_look);

        // Send the player list to the newly connected player.
        // (This is the list you see when you press tab in-game)
        let mut add_player_list = Vec::new();
        for (&uuid, player) in &self.online_players {
            add_player_list.push(CPlayerInfoUpdatePlayer {
                uuid,
                actions: CPlayerInfoActions {
                    add_player: Some(CPlayerInfoAddPlayer {
                        name: player.username.clone(),
                        properties: player.properties.clone(),
                    }),
                    update_gamemode: Some(player.gamemode.get_id()),
                    update_listed: Some(true),
                    ..Default::default()
                },
            });
        }
        add_player_list.push(CPlayerInfoUpdatePlayer {
            uuid: player.uuid,
            actions: CPlayerInfoActions {
                add_player: Some(CPlayerInfoAddPlayer {
                    name: player.username.clone(),
                    properties: player.properties.clone(),
                }),
                update_gamemode: Some(player.gamemode.get_id()),
                update_listed: Some(true),
                ..Default::default()
            },
        });

        let player_info = CPlayerInfoUpdate {
            players: add_player_list,
        }
        .encode();
        player.client.send_packet(&player_info);

        // Send the player's inventory
        let slot_data: Vec<Option<SlotData>> = player
            .inventory
            .iter()
            .map(|op| op.as_ref().map(utils::encode_slot_data))
            .collect();
        let window_items = CSetContainerContent {
            window_id: 0,
            state_id: 0,
            slot_data,
            carried_item: None,
        }
        .encode();
        player.client.send_packet(&window_items);

        // Send the player's selected item slot
        let held_item_change = CSetHeldItem {
            slot: player.selected_slot as i8,
        }
        .encode();
        player.client.send_packet(&held_item_change);

        player
            .client
            .send_packet(&self.shared.commands.declare_commands_packet());

        let time_update = UpdateTime {
            world_age: 0,
            // Noon
            time_of_day: -6000,
        }
        .encode();
        player.client.send_packet(&time_update);

        player.update_player_abilities();

        let game_event = CGameEvent {
            reason: CGameEventType::WaitForChunks,
            value: 0.0,
        }
        .encode();
        player.client.send_packet(&game_event);

        self.plot_sender
            .send(Message::PlayerJoined(player))
            .unwrap();
    }

    fn handle_player_login_start(&mut self, client_idx: usize, login_start: SLoginStart) {
        let clients = &mut self.network.handshaking_clients;
        let username = login_start.name;
        clients[client_idx].username = Some(username.clone());

        if self.shared.config.velocity.enabled {
            let message_id = rand::random();
            clients[client_idx].forwarding_message_id = Some(message_id);
            let plugin_message = CLoginPluginRequest {
                channel: "velocity:player_info".to_string(),
                message_id,
                data: vec![1], // MODERN_DEFAULT
            }
            .encode();
            clients[client_idx].send_packet(&plugin_message);
            return;
        }

        self.complete_player_login(client_idx);
    }

    fn complete_player_login(&mut self, client_idx: usize) {
        let clients = &mut self.network.handshaking_clients;
        let username = clients[client_idx].username.clone().unwrap();

        let set_compression = CSetCompression {
            threshold: COMPRESSION_THRESHOLD as i32,
        }
        .encode();
        clients[client_idx].send_packet(&set_compression);
        clients[client_idx].set_compressed(true);

        #[cfg(feature = "whitelist")]
        if let Some(whitelist) = &self.whitelist {
            // uuid will only be present if velocity is enabled in config
            let whitelisted = if let Some(uuid) = clients[client_idx].uuid {
                whitelist.iter().any(|entry| entry.uuid.0 == uuid)
            } else {
                whitelist.iter().any(|entry| entry.name == *username)
            };
            if !whitelisted {
                let disconnect = CDisconnectLogin {
                    reason: json!({
                        "text": "You are not whitelisted on this server"
                    })
                    .to_string(),
                }
                .encode();
                let client = clients.remove(client_idx);
                client.send_packet(&disconnect);
                client.close_connection();
                return;
            }
        }

        // Set generate uuid if it doesn't exist yet
        let uuid = match clients[client_idx].uuid {
            Some(uuid) => uuid,
            None => {
                let uuid = Player::generate_offline_uuid(&username);
                clients[client_idx].uuid = Some(uuid);
                uuid
            }
        };

        let login_success = CLoginSuccess {
            uuid,
            username,
            properties: clients[client_idx].properties.clone(),
        }
        .encode();
        clients[client_idx].send_packet(&login_success);
    }

    fn handle_message(&mut self, message: Message) {
        match message {
            Message::CommandSuggestions(mut suggestions, sender) => {
                suggestions.resolve(
                    SuggestionSource::PlayerNames,
                    self.online_players
                        .values()
                        .map(|player| player.username.as_str()),
                );
                let _ = sender.send(suggestions.into_response());
            }
            Message::PlayerJoined(player) => {
                info!("{} joined the game", player.username);
                // Send player info to plots
                let player_join_info = PlayerJoinInfo {
                    username: player.username.clone(),
                    uuid: player.uuid,
                    gamemode: player.gamemode,
                    properties: player.properties.clone(),
                };
                #[cfg(feature = "claims")]
                self.shared
                    .claims
                    .ensure_user(&format!("{:032x}", player.uuid), &player.username);
                self.broadcaster
                    .broadcast(BroadcastMessage::PlayerJoinedInfo(player_join_info));
                self.send_player_to_plot(player, true);
            }
            Message::PlayerLeft(uuid) => {
                if let Some((_, player)) = self.online_players.remove_entry(&uuid) {
                    info!("{} left the game", player.username);
                }
                self.broadcaster
                    .broadcast(BroadcastMessage::PlayerLeft(uuid));
            }
            Message::PlotUnload(plot_x, plot_z) => self.handle_plot_unload(plot_x, plot_z),
            Message::ChatInfo(uuid, username, message) => {
                info!("<{}> {}", username, message);
                self.broadcaster.broadcast(BroadcastMessage::Chat(
                    uuid,
                    TextComponent::from_legacy_text(
                        &self
                            .shared
                            .config
                            .chat_format
                            .replace("{username}", &username)
                            .replace("{message}", &message),
                    ),
                ));
            }
            Message::PlayerLeavePlot(player) => {
                self.send_player_to_plot(player, false);
            }
            Message::Shutdown => {
                self.graceful_shutdown();
            }
            Message::PlayerTeleportOther(player, target) => {
                if let Some((_, other_player)) =
                    self.online_players
                        .iter()
                        .find(|(uuid, other)| match &target {
                            PlayerTarget::Name(name) => other.username.eq_ignore_ascii_case(name),
                            PlayerTarget::Uuid(target) => **uuid == *target,
                            PlayerTarget::SelfPlayer => **uuid == player.uuid,
                        })
                {
                    let plot_x = other_player.plot_x;
                    let plot_z = other_player.plot_z;
                    let target_name = other_player.username.clone();

                    let plot_loaded = self
                        .running_plots
                        .iter()
                        .any(|p| p.plot_x == plot_x && p.plot_z == plot_z);
                    if !plot_loaded {
                        player
                            .send_system_message("Their plot wasn't loaded. How did this happen??");
                        self.send_player_to_plot(player, false);
                    } else {
                        self.update_player_entry(player.uuid, plot_x, plot_z);
                        let plot_list_entry = self
                            .running_plots
                            .iter()
                            .find(|p| p.plot_x == plot_x && p.plot_z == plot_z)
                            .unwrap();
                        let _ = plot_list_entry
                            .priv_message_sender
                            .send(PrivMessage::PlayerTeleportOther(player, target_name));
                    }
                } else {
                    player.send_system_message("Player not found!");
                    self.send_player_to_plot(player, false);
                }
            }
            Message::PlayerUpdateGamemode(uuid, gamemode) => {
                if let Some(player) = self.online_players.get_mut(&uuid) {
                    player.gamemode = gamemode;
                }
                self.broadcaster
                    .broadcast(BroadcastMessage::PlayerUpdateGamemode(uuid, gamemode));
            }
            #[cfg(feature = "whitelist")]
            Message::WhitelistAdd(uuid, username, sender) => {
                self.update_whitelist(sender, |whitelist| {
                    if whitelist.iter().any(|entry| entry.uuid.0 == uuid) {
                        return Err("That player is already whitelisted.");
                    }
                    let message = format!("{username} was successfully added to the whitelist.");
                    whitelist.push(WhitelistEntry {
                        name: username,
                        uuid: HyphenatedUUID(uuid),
                    });
                    Ok(message)
                })
            }
            #[cfg(feature = "whitelist")]
            Message::WhitelistRemove(uuid, sender) => self.update_whitelist(sender, |whitelist| {
                let Some(entry) = whitelist.iter().find(|entry| entry.uuid.0 == uuid) else {
                    return Err("That player is not whitelisted on this server.");
                };
                let message = format!(
                    "{} was successfully removed from the whitelist.",
                    entry.name
                );
                whitelist.retain(|entry| entry.uuid.0 != uuid);
                Ok(message)
            }),
        }
    }

    #[cfg(feature = "whitelist")]
    fn update_whitelist(
        &mut self,
        sender: PlayerPacketSender,
        change: impl FnOnce(&mut Vec<WhitelistEntry>) -> Result<String, &'static str>,
    ) {
        let Some(mut whitelist) = self.whitelist.clone() else {
            sender.send_error_message("Whitelist is not enabled!");
            return;
        };
        let message = match change(&mut whitelist) {
            Ok(message) => message,
            Err(message) => {
                sender.send_error_message(message);
                return;
            }
        };
        let result = serde_json::to_vec_pretty(&whitelist)
            .map_err(io::Error::from)
            .and_then(|data| {
                mchprs_save_data::replace_file("whitelist.json", |file| file.write_all(&data))
            });
        if let Err(error) = result {
            warn!("Failed to save whitelist: {error}");
            sender.send_error_message("Unable to save the whitelist. No changes were applied.");
            return;
        }
        self.whitelist = Some(whitelist);
        sender.send_system_message(&message);
    }

    fn update(&mut self) {
        while let Ok(message) = self.receiver.try_recv() {
            self.handle_message(message);
        }
        self.network.update();

        let mut client_idx = 0;
        let mut clients_len = self.network.handshaking_clients.len();
        loop {
            if client_idx >= clients_len {
                break;
            }

            let packets = self.network.handshaking_clients[client_idx].receive_packets();
            for packet in packets {
                packet.handle(self, client_idx);
                if self.network.handshaking_clients.len() != clients_len {
                    break;
                }
            }

            let new_len = self.network.handshaking_clients.len();

            if clients_len == new_len {
                client_idx += 1;
            }
            clients_len = new_len;
        }
    }
}

impl ServerBoundPacketHandler for MinecraftServer {
    fn handle_handshake(&mut self, handshake: SHandshake, client_idx: usize) {
        let clients = &mut self.network.handshaking_clients;
        let client = &mut clients[client_idx];
        let next_state = match handshake.next_state {
            1 => NetworkState::Status,
            2 => NetworkState::Login,
            // TODO: Handle invalid next state
            _ => return,
        };
        if next_state == NetworkState::Login && handshake.protocol_version != PROTOCOL_VERSION {
            warn!("A player tried to connect using the wrong version");
            let disconnect = CDisconnectLogin {
                reason: json!({ "text": format!("Version mismatch, I'm on {}!", MC_VERSION) })
                    .to_string(),
            }
            .encode();
            client.send_packet(&disconnect);
            client.close_connection();
        }
    }

    fn handle_request(&mut self, _request: SRequest, client_idk: usize) {
        let client = &mut self.network.handshaking_clients[client_idk];
        let response = CResponse {
            json_response: json!({
                "version": {
                    "name": MC_VERSION,
                    "protocol": PROTOCOL_VERSION
                },
                "players": {
                    "max": self.shared.config.max_players,
                    "online": self.online_players.len(),
                    "sample": []
                },
                "description": {
                    "text": self.shared.config.motd
                }
            })
            .to_string(),
        }
        .encode();
        client.send_packet(&response);
    }

    fn handle_ping(&mut self, ping: SPing, client_idx: usize) {
        let client = &mut self.network.handshaking_clients[client_idx];
        let pong = CPong {
            payload: ping.payload,
        }
        .encode();
        client.send_packet(&pong);
    }

    fn handle_login_start(&mut self, login_start: SLoginStart, client_idx: usize) {
        self.handle_player_login_start(client_idx, login_start);
    }

    fn handle_login_acknowledged(
        &mut self,
        _login_acknowledged: SLoginAcknowledged,
        client_idx: usize,
    ) {
        // The client has been switched to the Configuration state
        let client = &mut self.network.handshaking_clients[client_idx];

        // Sends the custom brand name to the player
        // (This can be seen in the f3 debug menu in-game)
        let brand = CConfigurationPluginMessage {
            channel: String::from("minecraft:brand"),
            data: {
                let mut data = Vec::new();
                data.write_string(32767, "Minecraft High Performance Redstone");
                data
            },
        }
        .encode();
        client.send_packet(&brand);

        let dimension = CRegistryDimensionType {
            fixed_time: Some(6000),
            has_skylight: true,
            has_ceiling: false,
            ultrawarm: false,
            natural: true,
            coordinate_scale: 1.0,
            bed_works: false,
            respawn_anchor_works: false,
            min_y: 0,
            height: PLOT_BLOCK_HEIGHT,
            logical_height: PLOT_BLOCK_HEIGHT,
            infiniburn: "#minecraft:infiniburn_overworld".to_owned(),
            effects: "#minecraft:overworld".to_owned(),
            ambient_light: 1.0,
            piglin_safe: false,
            has_raids: false,
            monster_spawn_block_light_limit: 0,
            monster_spawn_light_level: 0,
        };

        let codec = CRegistryDataCodec {
            dimension_types: HashMap::from([("minecraft:overworld".to_owned(), dimension.clone())]),
            biomes: HashMap::from([
                (
                    "mchprs:plot".to_owned(),
                    CRegistryBiome {
                        has_precipitation: false,
                        temperature: 0.5,
                        downfall: 0.5,
                        effects: CRegistryBiomeEffects {
                            sky_color: 0x7BA4FF,
                            water_fog_color: 0x050533,
                            fog_color: 0xC0D8FF,
                            water_color: 0x3F76E4,
                        },
                    },
                ),
                // Apparently the client NEEDS this to exist (MC-267103)
                (
                    "minecraft:plains".to_owned(),
                    CRegistryBiome {
                        has_precipitation: false,
                        temperature: 0.8,
                        downfall: 0.4,
                        effects: CRegistryBiomeEffects {
                            sky_color: 7907327,
                            water_fog_color: 329011,
                            fog_color: 12638463,
                            water_color: 4159204,
                        },
                    },
                ),
            ]),
        };
        let registry_data = CRegistryData {
            registry_codec: codec,
        };
        client.send_packet(&registry_data.encode());

        client.send_packet(&CFinishConfiguration.encode());
    }

    fn handle_acknowledge_finish_configuration(
        &mut self,
        _ackowledge_finish_configuration: SAcknowledgeFinishConfiguration,
        client_idx: usize,
    ) {
        self.handle_player_enter_play(client_idx);
    }

    fn handle_login_plugin_response(&mut self, packet: SLoginPluginResponse, client_idx: usize) {
        let clients = &mut self.network.handshaking_clients;

        if Some(packet.message_id) != clients[client_idx].forwarding_message_id {
            error!(
                "Unknown login plugin response with message id: {}",
                packet.message_id
            );
            return;
        }

        if !packet.successful {
            error!("Velocity forwarding channel not understood by client");
            return;
        }

        let velocity_response = VelocityResponseData::decode(&mut Cursor::new(&packet.data));

        let velocity_response = match velocity_response {
            Ok(velocity_response) => velocity_response,
            Err(err) => {
                error!("Could not decode velocity reponse data: {:?}", err);
                return;
            }
        };

        let secret = self.shared.config.velocity.secret.as_bytes();
        let mut mac = <Hmac<Sha256>>::new_from_slice(secret).unwrap();
        mac.update(&packet.data[32..]);
        if mac.verify_slice(&packet.data[..32]).is_err() {
            error!("Failed to verify velocity secret!");
            return;
        };

        clients[client_idx].uuid = Some(velocity_response.uuid);
        clients[client_idx].properties = velocity_response.properties;
        self.complete_player_login(client_idx);
    }
}

pub fn get_version_string() -> String {
    format!(
        "{} v{}\n{}",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION"),
        env!("GIT_HASH")
    )
}
