use super::{MinecraftServer, PlayerListEntry, PlotFailure, PrivMessage};
use crate::player::{PacketSender, Player};
use crate::plot::{Plot, PlotChannels};
use std::mem;
use std::sync::mpsc::{self, Receiver, SendError, Sender};
use std::sync::Arc;

pub struct PlotListEntry {
    plot_x: i32,
    plot_z: i32,
    priv_message_sender: Sender<PrivMessage>,
    always_loaded: bool,
    state: PlotState,
}

impl PlotListEntry {
    pub fn send(&mut self, message: PrivMessage) {
        match &mut self.state {
            PlotState::Running => {
                if let Err(SendError(message)) = self.priv_message_sender.send(message) {
                    self.state = PlotState::Unloading {
                        waiting: vec![message],
                    };
                }
            }
            PlotState::Unloading { waiting } => waiting.push(message),
        }
    }
}

pub enum PlotState {
    Running,
    Unloading { waiting: Vec<PrivMessage> },
}

impl MinecraftServer {
    pub(super) fn running_plot(&mut self, plot_x: i32, plot_z: i32) -> Option<&mut PlotListEntry> {
        self.running_plots
            .iter_mut()
            .find(|p| p.plot_x == plot_x && p.plot_z == plot_z)
    }

    pub(super) fn handle_plot_idle(&mut self, plot_x: i32, plot_z: i32) {
        let Some(entry) = self.running_plot(plot_x, plot_z) else {
            return;
        };
        if entry.always_loaded || !matches!(entry.state, PlotState::Running) {
            return;
        }
        let _ = entry.priv_message_sender.send(PrivMessage::Unload);
        entry.state = PlotState::Unloading {
            waiting: Vec::new(),
        };
    }

    pub(super) fn remove_plot(&mut self, plot_x: i32, plot_z: i32) -> Option<PlotListEntry> {
        let index = self
            .running_plots
            .iter()
            .position(|p| p.plot_x == plot_x && p.plot_z == plot_z)?;
        Some(self.running_plots.remove(index))
    }

    pub(super) fn handle_unload_refused(&mut self, plot_x: i32, plot_z: i32) {
        let Some(entry) = self.running_plot(plot_x, plot_z) else {
            return;
        };
        let PlotState::Unloading { waiting } = mem::replace(&mut entry.state, PlotState::Running)
        else {
            return;
        };
        for message in waiting {
            entry.send(message);
        }
    }

    pub(super) fn handle_plot_exited(
        &mut self,
        plot_x: i32,
        plot_z: i32,
        players: Vec<Player>,
        private: Receiver<PrivMessage>,
        failure: Option<PlotFailure>,
    ) {
        let Some(entry) = self.remove_plot(plot_x, plot_z) else {
            return;
        };
        let always_loaded = entry.always_loaded;
        let waiting = match entry.state {
            PlotState::Unloading { waiting } => waiting,
            PlotState::Running => Vec::new(),
        };
        let transfers: Vec<PrivMessage> = private
            .try_iter()
            .chain(waiting)
            .filter(|message| !matches!(message, PrivMessage::Unload))
            .collect();
        match failure {
            None if !transfers.is_empty() => {
                self.spawn_plot(plot_x, plot_z, always_loaded, None);
                if let Some(entry) = self.running_plot(plot_x, plot_z) {
                    transfers
                        .into_iter()
                        .for_each(|message| entry.send(message));
                }
            }
            None => {}
            Some(failure) => {
                for mut player in transfers.into_iter().filter_map(PrivMessage::into_player) {
                    player.send_error_message(failure.message());
                    Plot::send_player_away(plot_x, plot_z, &mut player);
                    self.send_player_to_plot(player, false);
                }
            }
        }
        for player in players {
            self.send_player_to_plot(player, false);
        }
    }

    pub(super) fn spawn_plot(
        &mut self,
        plot_x: i32,
        plot_z: i32,
        always_loaded: bool,
        initial_player: Option<Player>,
    ) {
        let (priv_message_sender, private) = mpsc::channel();
        let channels = PlotChannels {
            broadcasts: self.broadcaster.add_rx(),
            server: self.plot_sender.clone(),
            private,
        };
        let shared = Arc::clone(&self.shared);
        Plot::load_and_run(plot_x, plot_z, channels, initial_player, shared);
        self.running_plots.push(PlotListEntry {
            plot_x,
            plot_z,
            priv_message_sender,
            always_loaded,
            state: PlotState::Running,
        });
    }

    pub(super) fn send_player_to_plot(&mut self, player: Player, new_entry: bool) {
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

        match self.running_plot(plot_x, plot_z) {
            Some(entry) => entry.send(PrivMessage::PlayerEnterPlot(player)),
            None => self.spawn_plot(plot_x, plot_z, false, Some(player)),
        }
    }
}
