use super::{Plot, ERROR_REDPILER_ACTIVE};
use crate::interaction;
use crate::interaction::UseOnBlockContext;
use crate::player::PacketSender;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::items::Item;
use mchprs_blocks::{BlockFace, BlockPos};
use mchprs_network::packets::clientbound::{CBlockUpdate, CWorldEvent, ClientBoundPacket};
use mchprs_network::packets::serverbound::SUseItemOn;
use mchprs_redstone::{set_block, Notify};

impl Plot {
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

    pub(super) fn handle_use_item_impl(&mut self, use_item_on: &SUseItemOn, player: usize) {
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
            let same = self.players[player].second_position == Some(block_pos);
            if !same {
                self.players[player].worldedit_set_second_position(block_pos);
            }
            cancel(self);
            return;
        }

        let block = self.world.block(block_pos);
        if !self.simulation.is_editable() {
            if !self.players[player].crouching && self.use_block_in_circuit(block_pos) {
                self.play_use_sound(player, block_pos, block);
                return;
            }
            if use_item_on.hand == 0 {
                self.players[player].send_error_message(ERROR_REDPILER_ACTIVE);
            }
            cancel(self);
            return;
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
        } else if !self.players[player].crouching {
            interaction::on_use(
                block,
                &mut self.world,
                &mut self.players[player],
                block_pos,
                None,
            );
        }
        if self.world.block(block_pos) != block {
            self.play_use_sound(player, block_pos, block);
        }
        self.world.flush_block_changes();
        self.send_sounds();
    }

    pub(super) fn handle_player_digging(&mut self, block_pos: BlockPos, player: usize) {
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
            if let Some(pos) = self.players[player].first_position
                && pos == block_pos
            {
                return;
            }
            self.players[player].worldedit_set_first_position(block_pos);
            return;
        }

        if !self.simulation.is_editable() {
            self.players[player].send_error_message(ERROR_REDPILER_ACTIVE);
            self.send_block_corrections(&[block_pos]);
            return;
        }

        set_block(&mut self.world, block_pos, Block::Air, Notify::All);
        self.world.flush_block_changes();
        self.send_sounds();

        let effect = CWorldEvent {
            event: 2001,
            x: block_pos.x,
            y: block_pos.y,
            z: block_pos.z,
            data: block.get_id() as i32,
            disable_relative_volume: false,
        }
        .encode();
        for (other_idx, other_player) in self.players.iter().enumerate() {
            if player == other_idx {
                continue;
            };
            other_player.client.send_packet(&effect);
        }
    }
}
