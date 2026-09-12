use crate::commands::argument_parser::IntegerParser;
use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandError, CommandResult};
use crate::commands::registry::CommandRegistry;
use crate::plot::PLOT_BLOCK_HEIGHT;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::BlockPos;
use mchprs_world::World;

pub(super) fn register(registry: &mut CommandRegistry) {
    let mut up = Command::new("up").alias("u");
    let distance = up.optional("distance", IntegerParser::new(1, 100));
    registry.register(up.executes(move |ctx| {
        let distance = ctx.get(distance).unwrap_or(1);

        let mut new_pos = ctx.player()?.pos;
        new_pos.y += distance as f64;

        let block_pos = new_pos.block_pos();
        let platform_pos = BlockPos::new(block_pos.x, block_pos.y - 1, block_pos.z);
        if matches!(ctx.world().get_block(platform_pos), Block::Air) {
            ctx.world_mut().set_block(platform_pos, Block::Glass);
        }

        ctx.teleport(new_pos)?;
        ctx.worldedit_message(&format!("Moved up {distance} blocks"))
    }));

    let mut ascend = Command::new("ascend").alias("asc");
    let levels = ascend.optional("levels", IntegerParser::new(1, 100));
    registry.register(ascend.executes(move |ctx| {
        let levels = ctx.get(levels).unwrap_or(1);
        let y = ctx.player()?.pos.block_pos().y;
        exec_floors(
            ctx,
            levels,
            y + 1..=PLOT_BLOCK_HEIGHT,
            "Ascended",
            "No free spot above you found.",
        )
    }));

    let mut descend = Command::new("descend").alias("desc");
    let levels = descend.optional("levels", IntegerParser::new(1, 100));
    registry.register(descend.executes(move |ctx| {
        let levels = ctx.get(levels).unwrap_or(1);
        let y = ctx.player()?.pos.block_pos().y;
        exec_floors(
            ctx,
            levels,
            (1..y).rev(),
            "Descended",
            "No free spot below you found.",
        )
    }));
}

fn exec_floors(
    ctx: &mut ExecutionContext<'_>,
    levels: i32,
    heights: impl Iterator<Item = i32>,
    verb: &str,
    error: &str,
) -> CommandResult<()> {
    let player_pos = ctx.player()?.pos.block_pos();
    let world = ctx.world();

    let mut player_y = player_pos.y;
    let mut remaining = levels;
    for y in heights {
        if remaining == 0 {
            break;
        }
        let floor_pos = BlockPos::new(player_pos.x, y - 1, player_pos.z);
        let pos = BlockPos::new(player_pos.x, y, player_pos.z);
        let high_pos = BlockPos::new(player_pos.x, y + 1, player_pos.z);

        if world.get_block(floor_pos) != Block::Air
            && world.get_block(pos) == Block::Air
            && world.get_block(high_pos) == Block::Air
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
    ctx.teleport(new_pos)?;
    ctx.worldedit_message(&format!("{verb} {} levels", levels - remaining))
}
