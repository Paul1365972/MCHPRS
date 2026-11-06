use super::Plot;
use crate::player::Player;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::BlockPos;
use mchprs_network::packets::clientbound::{CSoundEffect, ClientBoundPacket};
use mchprs_network::packets::PacketEncoder;
use mchprs_redstone::sound::Sound;

const SOUND_BUDGET_PER_PLAYER_PER_SECOND: f32 = 100.0;

impl Plot {
    pub(super) fn send_sounds(&mut self) {
        let limit =
            (SOUND_BUDGET_PER_PLAYER_PER_SECOND / self.world.world_send_rate.0).ceil() as usize;
        let sounds: Vec<_> = self.world.pending_sounds.drain().collect();
        for player in &self.players {
            let mut heard: Vec<_> = sounds
                .iter()
                .map(|&(pos, sound)| (distance(player, pos), pos, sound))
                .filter(|&(distance, _, sound)| distance < sound.range())
                .collect();
            if heard.len() > limit {
                heard.select_nth_unstable_by(limit, |a, b| a.0.total_cmp(&b.0));
                heard.truncate(limit);
            }
            for (_, pos, sound) in heard {
                player.client.send_packet(&encode_sound(pos, sound));
            }
        }
    }

    pub(super) fn play_use_sound(&self, user_idx: usize, pos: BlockPos, block: Block) {
        if self.world.world_send_rate.0 == 0.0 {
            return;
        }
        let Some(sound) = Sound::of_use(block) else {
            return;
        };
        for (idx, player) in self.players.iter().enumerate() {
            if idx != user_idx && distance(player, pos) < sound.range() {
                player.client.send_packet(&encode_sound(pos, sound));
            }
        }
    }
}

fn distance(player: &Player, pos: BlockPos) -> f64 {
    let dx = player.pos.x - (f64::from(pos.x) + 0.5);
    let dy = player.pos.y - (f64::from(pos.y) + 0.5);
    let dz = player.pos.z - (f64::from(pos.z) + 0.5);
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn encode_sound(pos: BlockPos, sound: Sound) -> PacketEncoder {
    CSoundEffect {
        sound_id: sound.event + 1,
        sound_category: sound.source as i32,
        x: pos.x * 8 + 4,
        y: pos.y * 8 + 4,
        z: pos.z * 8 + 4,
        volume: sound.volume,
        pitch: sound.pitch,
        seed: rand::random(),
    }
    .encode()
}
