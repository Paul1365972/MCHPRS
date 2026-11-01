use super::Plot;
use crate::player::PacketSender;
use crate::profile::PlayerProfile;
use crate::server::Message;
use mchprs_network::PlayerPacketSender;
use std::thread;
use tracing::warn;

impl Plot {
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

    pub fn whitelist_remove(&self, username: String, packet_sender: PlayerPacketSender) {
        if !self.shared.config.whitelist {
            packet_sender.send_error_message("Whitelist is not enabled!");
            return;
        }
        self.look_up_profile(username, packet_sender, |profile, packet_sender| {
            Message::WhitelistRemove(profile.uuid.0, packet_sender)
        });
    }

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
}
