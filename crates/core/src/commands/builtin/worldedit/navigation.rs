use crate::{
    commands::{
        argument::ArgumentType,
        context::ExecutionContext,
        define::Command,
        error::{CommandError, CommandResult},
        registry::CommandRegistry,
    },
    player::PlayerPos,
    plot::PLOT_BLOCK_HEIGHT,
};
use mchprs_blocks::{
    block_store::{BlockStore, BlockView},
    blocks::Block,
    BlockPos,
};

fn free_spot_above(world: &impl BlockView, pos: BlockPos) -> PlayerPos {
    let is_air = |y| world.block(BlockPos::new(pos.x, y, pos.z)) == Block::Air;
    let y = (pos.y..PLOT_BLOCK_HEIGHT)
        .find(|&y| is_air(y) && is_air(y + 1))
        .unwrap_or(pos.y.max(PLOT_BLOCK_HEIGHT));
    PlayerPos::new(pos.x as f64 + 0.5, y as f64, pos.z as f64 + 0.5)
}

pub(super) fn register(registry: &mut CommandRegistry) {
    registry.register(Command::new("jumpto").alias("j").executes(|ctx| {
        let target = ctx.target_block(1000.0)?;
        let above = BlockPos::new(target.x, target.y + 1, target.z);
        let pos = free_spot_above(ctx.world(), above);
        ctx.player_mut()?.teleport(pos);
        ctx.worldedit_message("Teleported to block.")
    }));

    registry.register(Command::new("unstuck").alias("!").executes(|ctx| {
        let feet = ctx.player()?.pos.block_pos();
        let pos = free_spot_above(ctx.world(), feet);
        ctx.player_mut()?.teleport(pos);
        ctx.worldedit_message("Moved to free position.")
    }));

    registry.register(
        Command::new("up")
            .alias("u")
            .permission("worldedit.navigation.up")
            .require_plot_ownership()
            .optional("distance", ArgumentType::integer(1, 100))
            .flag('f', "force-flight", "Force using flight to keep you still")
            .flag('g', "force-glass", "Force using glass to keep you still")
            .executes(|ctx| {
                let distance = ctx.arg_or("distance", 1)?;
                let force_flight = ctx.flag("force-flight");
                let force_glass = ctx.flag("force-glass");

                let player = ctx.player_mut()?;
                let mut new_pos = player.pos;
                new_pos.y += distance as f64;

                if force_flight {
                    player.flying = true;
                    player.update_player_abilities();
                }

                if force_glass || !force_flight {
                    let block_pos = new_pos.block_pos();
                    let platform_pos = BlockPos::new(block_pos.x, block_pos.y - 1, block_pos.z);
                    if matches!(ctx.world().block(platform_pos), Block::Air) {
                        ctx.world_mut()?.set_block(platform_pos, Block::Glass);
                    }
                }

                ctx.player_mut()?.teleport(new_pos);
                ctx.worldedit_message(&format!("Moved up {} blocks", distance))
            }),
    );

    fn exec_floors(
        ctx: &mut ExecutionContext<'_>,
        ys: impl Iterator<Item = i32>,
        verb: &str,
        error: &str,
    ) -> CommandResult<()> {
        let levels: i32 = ctx.arg_or("levels", 1)?;
        let player_pos = ctx.player()?.pos.block_pos();
        let world = ctx.world();

        let mut player_y = player_pos.y;
        let mut remaining = levels;
        for y in ys {
            if remaining == 0 {
                break;
            }
            let floor_pos = BlockPos::new(player_pos.x, y - 1, player_pos.z);
            let pos = BlockPos::new(player_pos.x, y, player_pos.z);
            let high_pos = BlockPos::new(player_pos.x, y + 1, player_pos.z);

            if world.block(floor_pos) != Block::Air
                && world.block(pos) == Block::Air
                && world.block(high_pos) == Block::Air
            {
                player_y = y;
                remaining -= 1;
            }
        }

        if player_y == player_pos.y {
            return Err(CommandError::runtime(error));
        }

        let mut new_pos = ctx.player()?.pos;
        new_pos.y = player_y as f64;
        ctx.player_mut()?.teleport(new_pos);
        ctx.worldedit_message(&format!("{} {} levels", verb, levels - remaining))
    }

    registry.register(
        Command::new("ascend")
            .alias("asc")
            .permission("worldedit.navigation.ascend")
            .require_plot_ownership()
            .optional("levels", ArgumentType::integer(1, 100))
            .executes(|ctx| {
                let y = ctx.player()?.pos.block_pos().y;
                exec_floors(
                    ctx,
                    y + 1..=PLOT_BLOCK_HEIGHT,
                    "Ascended",
                    "No free spot above you found.",
                )
            }),
    );

    registry.register(
        Command::new("descend")
            .alias("desc")
            .permission("worldedit.navigation.descend")
            .require_plot_ownership()
            .optional("levels", ArgumentType::integer(1, 100))
            .executes(|ctx| {
                let y = ctx.player()?.pos.block_pos().y;
                exec_floors(
                    ctx,
                    (1..y).rev(),
                    "Descended",
                    "No free spot below you found.",
                )
            }),
    );
}
