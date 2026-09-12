use super::selection::update_selection;
use crate::{
    commands::{
        argument::ArgumentType,
        context::ExecutionContext,
        define::Command,
        error::{CommandError, CommandResult},
        registry::CommandRegistry,
        value::{Direction, DirectionWithDiagonals},
    },
    plot::{PLOT_BLOCK_HEIGHT, PLOT_BLOCK_WIDTH},
    worldedit::{calculate_expanded_selection, calculate_selection_volume},
};
use mchprs_blocks::{BlockFacing, BlockPos};
use mchprs_schematic::{create_clipboard, paste_clipboard};
use std::time::Instant;

const MAX_STACK_VOLUME: i64 =
    PLOT_BLOCK_WIDTH as i64 * PLOT_BLOCK_WIDTH as i64 * PLOT_BLOCK_HEIGHT as i64;

pub(super) fn register(registry: &mut CommandRegistry) {
    registry.register(
        Command::new("/stack")
            .alias("/s")
            .permission("worldedit.region.stack")
            .require_plot_ownership()
            .optional("count", ArgumentType::integer(1, 1000))
            .optional("direction", ArgumentType::Direction)
            .flag('a', "ignore-air", "Stack without air blocks")
            .flag(
                's',
                "shift-selection",
                "Shift the selection to the last stacked copy",
            )
            .executes(|ctx| {
                let direction = ctx.arg_or("direction", Direction::Me)?;
                let direction = direction.resolve(ctx.player()?.get_facing());
                let (first_pos, second_pos) = ctx.get_selection()?;
                let size = first_pos.max(second_pos) - first_pos.min(second_pos);
                let spacing = match direction {
                    BlockFacing::North | BlockFacing::South => size.z + 1,
                    BlockFacing::East | BlockFacing::West => size.x + 1,
                    BlockFacing::Up | BlockFacing::Down => size.y + 1,
                };
                let offset = direction.offset_pos(BlockPos::zero(), spacing);
                exec_stack(ctx, (first_pos, second_pos), offset, ctx.flag("ignore-air"))
            }),
    );
    registry.add_custom_alias("/sa", "/stack {} -a");

    registry.register(
        Command::new("/rstack")
            .alias("/rs")
            .permission("redstonetools.rstack")
            .require_plot_ownership()
            .optional("count", ArgumentType::integer(1, 1000))
            .optional("direction", ArgumentType::DirectionWithDiagonals)
            .optional("offset", ArgumentType::integer(1, 1000))
            .flag('w', "with-air", "Stack with air blocks")
            .flag(
                's',
                "shift-selection",
                "Shift the selection to the last stacked copy",
            )
            .flag(
                'e',
                "expand-selection",
                "Expand selection to include stacked region",
            )
            .executes(|ctx| {
                let direction = ctx.arg_or("direction", DirectionWithDiagonals::Me)?;
                let spacing = ctx.arg_or("offset", 2)?;
                let player = ctx.player()?;
                let direction = direction.resolve(player.get_facing(), player.pitch);
                let selection = ctx.get_selection()?;
                exec_stack(ctx, selection, direction * spacing, !ctx.flag("with-air"))
            }),
    );
}

fn check_stack_volume(first_pos: BlockPos, second_pos: BlockPos, count: i32) -> CommandResult<()> {
    let volume = i64::from(calculate_selection_volume(first_pos, second_pos)) * i64::from(count);
    if volume > MAX_STACK_VOLUME {
        return Err(CommandError::runtime(format!(
            "Stacking {count} copies would change {volume} blocks, but the limit is {MAX_STACK_VOLUME}."
        )));
    }
    Ok(())
}

fn exec_stack(
    ctx: &mut ExecutionContext<'_>,
    selection: (BlockPos, BlockPos),
    offset: BlockPos,
    ignore_air: bool,
) -> CommandResult<()> {
    let count = ctx.arg_or("count", 1)?;
    let shift_selection = ctx.flag("shift-selection");
    let expand_selection = ctx.flag("expand-selection");
    if shift_selection && expand_selection {
        return Err(CommandError::runtime(
            "Use either --shift-selection or --expand-selection, not both.",
        ));
    }

    let (first_pos, second_pos) = selection;
    check_stack_volume(first_pos, second_pos, count)?;

    let start_time = Instant::now();
    let destinations = (1..=count).rev().map(|i| {
        let offset = offset * i;
        (first_pos + offset, second_pos + offset)
    });
    let clipped = ctx.capture_undo_regions(destinations, first_pos)?;
    let clipboard = create_clipboard(ctx.world(), first_pos, first_pos, second_pos);

    for i in 1..=count {
        let destination = (first_pos + offset * i, second_pos + offset * i);
        if ctx.clip_region(destination.0, destination.1).is_none() {
            continue;
        }
        paste_clipboard(ctx.world_mut(), &clipboard, destination.0, ignore_air);
    }

    let offset = offset * count;
    if shift_selection {
        update_selection(ctx, selection, (first_pos + offset, second_pos + offset))?;
    } else if expand_selection {
        let expanded = calculate_expanded_selection(first_pos, second_pos, offset, false);
        update_selection(ctx, selection, expanded)?;
    }

    let message = if clipped {
        "Your selection was stacked (clipped to plot bounds)."
    } else {
        "Your selection was stacked."
    };
    ctx.worldedit_message(&format!("{} ({:?})", message, start_time.elapsed()))
}
