use super::argument::{Arguments, Handle};
use super::error::{CommandError, CommandResult, InternalError};
use super::value::{BlockCoordinates, PositionCoordinates};
use crate::player::{PacketSender, Player, PlayerPos};
use crate::plot::{Plot, PlotWorld, PLOT_BLOCK_HEIGHT};
use crate::shared::Shared;
use crate::worldedit::{ray_trace_block, WorldEditUndo};
use mchprs_blocks::BlockPos;
use mchprs_network::text::TextComponent;
use mchprs_schematic::{create_clipboard, WorldEditClipboard};

const WORLD_LIMIT_HORIZONTAL: f64 = 30_000_000.0;
const WORLD_LIMIT_VERTICAL: f64 = 20_000_000.0;

pub(super) struct ExecutionContext<'a> {
    plot: &'a mut Plot,
    player_idx: usize,
    arguments: &'a Arguments,
    world_access: WorldAccess,
}

impl<'a> ExecutionContext<'a> {
    pub(super) fn new(plot: &'a mut Plot, player_idx: usize, arguments: &'a Arguments) -> Self {
        Self {
            plot,
            player_idx,
            arguments,
            world_access: WorldAccess::Unobserved,
        }
    }

    pub(super) fn get<H: Handle>(&self, handle: H) -> H::Value {
        handle.read(self.arguments)
    }

    pub(super) fn position(&self, coordinates: PositionCoordinates) -> CommandResult<PlayerPos> {
        let player = self.player()?;
        let position = player.pos;
        let (x, y, z) = coordinates.resolve(
            (position.x, position.y, position.z),
            player.yaw,
            player.pitch,
        )?;
        Ok(PlayerPos::new(x, y, z))
    }

    pub(super) fn block_position(&self, coordinates: BlockCoordinates) -> CommandResult<BlockPos> {
        let player = self.player()?;
        let position = player.pos;
        coordinates.resolve(
            (position.x, position.y, position.z),
            player.yaw,
            player.pitch,
        )
    }

    pub(super) fn reply(&self, message: &str) -> CommandResult<()> {
        self.player()?.send_system_message(message);
        Ok(())
    }

    pub(super) fn reply_legacy(&self, message: &str) -> CommandResult<()> {
        let components = TextComponent::from_legacy_text(message);
        self.player()?.send_chat_message(&components);
        Ok(())
    }

    pub(super) fn plot(&self) -> &Plot {
        self.plot
    }

    pub(super) fn plot_mut(&mut self) -> &mut Plot {
        self.plot
    }

    pub(super) fn shared(&self) -> &Shared {
        &self.plot.shared
    }

    pub(super) fn world(&mut self) -> &PlotWorld {
        if self.world_access == WorldAccess::Unobserved {
            if self.plot.redpiler.is_active() {
                self.plot.redpiler.flush_all(&mut self.plot.world);
            }
            self.world_access = WorldAccess::Synchronized;
        }
        &self.plot.world
    }

    pub(super) fn mutated_world(&self) -> bool {
        self.world_access == WorldAccess::Mutable
    }

    pub(super) fn world_mut(&mut self) -> &mut PlotWorld {
        self.begin_edit();
        &mut self.plot.world
    }

    pub(super) fn world_and_player_mut(&mut self) -> CommandResult<(&mut PlotWorld, &mut Player)> {
        self.begin_edit();
        let index = self.player_idx;
        let plot = &mut *self.plot;
        let player = plot
            .players
            .get_mut(index)
            .ok_or(InternalError::InvalidPlayerIndex { index })?;
        Ok((&mut plot.world, player))
    }

    fn begin_edit(&mut self) {
        if self.world_access != WorldAccess::Mutable {
            self.world();
            self.plot.reset_redpiler();
            self.world_access = WorldAccess::Mutable;
        }
    }

    pub(super) fn player(&self) -> Result<&Player, InternalError> {
        let index = self.player_idx;
        self.plot
            .players
            .get(index)
            .ok_or(InternalError::InvalidPlayerIndex { index })
    }

    pub(super) fn player_mut(&mut self) -> Result<&mut Player, InternalError> {
        let index = self.player_idx;
        self.plot
            .players
            .get_mut(index)
            .ok_or(InternalError::InvalidPlayerIndex { index })
    }

    pub(super) fn teleport(&mut self, pos: PlayerPos) -> CommandResult<()> {
        let PlayerPos { x, y, z } = pos;
        if x.abs() > WORLD_LIMIT_HORIZONTAL
            || z.abs() > WORLD_LIMIT_HORIZONTAL
            || y.abs() > WORLD_LIMIT_VERTICAL
        {
            return Err(CommandError::coordinate_out_of_range());
        }
        self.player()?;
        self.plot.teleport_player(self.player_idx, pos);
        Ok(())
    }

    pub(super) fn player_idx(&self) -> usize {
        self.player_idx
    }

    pub(super) fn target_block(&mut self, max_distance: f64) -> CommandResult<BlockPos> {
        let player = self.player()?;
        let (pos, pitch, yaw) = (player.pos, player.pitch as f64, player.yaw as f64);
        ray_trace_block(self.world(), pos, pitch, yaw, max_distance)
            .ok_or_else(|| CommandError::runtime("No block in sight"))
    }

    pub(super) fn selection(&self) -> CommandResult<(BlockPos, BlockPos)> {
        let plot_x = self.plot.world.x;
        let plot_z = self.plot.world.z;
        let player = self.player()?;

        let (first, second) = match (player.first_position, player.second_position) {
            (Some(first), Some(second)) => (first, second),
            _ => return Err(CommandError::runtime("Make a region selection first.")),
        };

        for (position, pos) in [("First", first), ("Second", second)] {
            if !Plot::in_plot_bounds(plot_x, plot_z, pos.x, pos.z)
                || !(0..PLOT_BLOCK_HEIGHT).contains(&pos.y)
            {
                return Err(CommandError::runtime(format!(
                    "{position} position is outside plot bounds"
                )));
            }
        }

        Ok((first, second))
    }

    pub(super) fn clip_region(
        &self,
        first: BlockPos,
        second: BlockPos,
    ) -> Option<(BlockPos, BlockPos)> {
        let (plot_min, plot_max) = self.plot.world.get_corners();
        let min = first.min(second).max(plot_min);
        let max = first.max(second).min(plot_max);
        (min.x <= max.x && min.y <= max.y && min.z <= max.z).then_some((min, max))
    }

    pub(super) fn capture_undo_regions(
        &mut self,
        regions: impl IntoIterator<Item = (BlockPos, BlockPos)>,
        origin: BlockPos,
    ) -> CommandResult<bool> {
        let mut clipped = false;
        let regions: Vec<_> = regions
            .into_iter()
            .filter_map(|(first, second)| {
                let region = self.clip_region(first, second);
                clipped |= region != Some((first.min(second), first.max(second)));
                region
            })
            .collect();
        if regions.is_empty() {
            return Err(CommandError::destination_outside_plot());
        }
        let (world, player) = self.world_and_player_mut()?;
        let clipboards = regions
            .into_iter()
            .map(|(first_pos, second_pos)| create_clipboard(world, origin, first_pos, second_pos))
            .collect();

        let undo = WorldEditUndo {
            clipboards,
            pos: origin,
            plot_x: world.x,
            plot_z: world.z,
        };

        player.worldedit_undo.push(undo);
        player.worldedit_redo.clear();
        Ok(clipped)
    }

    pub(super) fn worldedit_message(&self, message: &str) -> CommandResult<()> {
        self.player()?.send_worldedit_message(message);
        Ok(())
    }

    pub(super) fn clipboard(&self) -> CommandResult<&WorldEditClipboard> {
        self.player()?
            .worldedit_clipboard
            .as_ref()
            .ok_or_else(CommandError::empty_clipboard)
    }

    pub(super) fn set_clipboard(&mut self, clipboard: WorldEditClipboard) -> CommandResult<()> {
        self.player_mut()?.worldedit_clipboard = Some(clipboard);
        Ok(())
    }
}

#[derive(PartialEq)]
enum WorldAccess {
    Unobserved,
    Synchronized,
    Mutable,
}
