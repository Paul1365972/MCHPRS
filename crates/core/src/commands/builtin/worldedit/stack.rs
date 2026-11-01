use super::selection::update_selection;
use crate::commands::argument_parser::{DiagonalDirectionParser, DirectionParser, IntegerParser};
use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandError, CommandResult};
use crate::commands::registry::CommandRegistry;
use crate::commands::value::{Direction, DirectionWithDiagonals};
use crate::plot::{PLOT_BLOCK_HEIGHT, PLOT_BLOCK_WIDTH};
use crate::worldedit::{calculate_expanded_selection, calculate_selection_volume};
use mchprs_blocks::{BlockFacing, BlockPos};
use mchprs_schematic::{create_clipboard, paste_clipboard};
use mchprs_syntax::flag::Flag;
use std::time::Instant;

const IGNORE_AIR: Flag = Flag::switch("ignore-air", Some('a'), "Stack without air blocks");
const SHIFT_SELECTION: Flag = Flag::switch(
    "shift-selection",
    Some('s'),
    "Shift the selection to the last stacked copy",
);
const WITH_AIR: Flag = Flag::switch("with-air", Some('w'), "Stack with air blocks");
const EXPAND_SELECTION: Flag = Flag::switch(
    "expand-selection",
    Some('e'),
    "Expand selection to include stacked region",
);

const SELECTION_UPDATES: [(Flag, SelectionUpdate); 2] = [
    (SHIFT_SELECTION, SelectionUpdate::Shift),
    (EXPAND_SELECTION, SelectionUpdate::Expand),
];

const MAX_STACK_VOLUME: i64 =
    PLOT_BLOCK_WIDTH as i64 * PLOT_BLOCK_WIDTH as i64 * PLOT_BLOCK_HEIGHT as i64;

#[derive(Clone, Copy)]
enum SelectionUpdate {
    Keep,
    Shift,
    Expand,
}

pub(super) fn register(registry: &mut CommandRegistry) {
    let mut stack = Command::new("/stack").alias("/s");
    let count = stack.optional("count", IntegerParser::new(1, 1000));
    let direction = stack.optional("direction", DirectionParser);
    let ignore_air = stack.flag(IGNORE_AIR);
    let shift_selection = stack.flag(SHIFT_SELECTION);
    registry.register(stack.executes(move |ctx| {
        let direction = ctx.get(direction).unwrap_or(Direction::Me);
        let direction = direction.resolve(ctx.player()?.get_facing());
        let (first_pos, second_pos) = ctx.selection()?;
        let size = first_pos.max(second_pos) - first_pos.min(second_pos);
        let spacing = match direction {
            BlockFacing::North | BlockFacing::South => size.z + 1,
            BlockFacing::East | BlockFacing::West => size.x + 1,
            BlockFacing::Up | BlockFacing::Down => size.y + 1,
        };
        let stacking = Stacking {
            count: ctx.get(count).unwrap_or(1),
            offset: direction.offset_pos(BlockPos::zero(), spacing),
            ignore_air: ctx.get(ignore_air),
            selection_update: if ctx.get(shift_selection) {
                SelectionUpdate::Shift
            } else {
                SelectionUpdate::Keep
            },
        };
        exec_stack(ctx, (first_pos, second_pos), stacking)
    }));
    registry.alias("/sa", "/stack -a");

    let mut rstack = Command::new("/rstack").alias("/rs");
    let count = rstack.optional("count", IntegerParser::new(1, 1000));
    let direction = rstack.optional("direction", DiagonalDirectionParser);
    let spacing = rstack.optional("offset", IntegerParser::new(1, 1000));
    let with_air = rstack.flag(WITH_AIR);
    let selection_update = rstack.one_of(SELECTION_UPDATES);
    registry.register(rstack.executes(move |ctx| {
        let direction = ctx.get(direction).unwrap_or(DirectionWithDiagonals::Me);
        let player = ctx.player()?;
        let direction = direction.resolve(player.get_facing(), player.pitch);
        let selection = ctx.selection()?;
        let stacking = Stacking {
            count: ctx.get(count).unwrap_or(1),
            offset: direction * ctx.get(spacing).unwrap_or(2),
            ignore_air: !ctx.get(with_air),
            selection_update: ctx.get(selection_update).unwrap_or(SelectionUpdate::Keep),
        };
        exec_stack(ctx, selection, stacking)
    }));
}

struct Stacking {
    count: i32,
    offset: BlockPos,
    ignore_air: bool,
    selection_update: SelectionUpdate,
}

fn exec_stack(
    ctx: &mut ExecutionContext<'_>,
    selection: (BlockPos, BlockPos),
    stacking: Stacking,
) -> CommandResult<()> {
    let Stacking {
        count,
        offset,
        ignore_air,
        selection_update,
    } = stacking;
    let (first_pos, second_pos) = selection;
    check_stack_volume(first_pos, second_pos, count)?;

    let start_time = Instant::now();
    let destinations = (1..=count).rev().map(|i| {
        let offset = offset * i;
        (first_pos + offset, second_pos + offset)
    });
    let clipped = ctx.capture_undo_regions(destinations, first_pos)?;
    let clipboard = create_clipboard(ctx.world(), first_pos, first_pos, second_pos);

    let mut last_copy = None;
    for i in 1..=count {
        let destination = (first_pos + offset * i, second_pos + offset * i);
        if ctx.clip_region(destination.0, destination.1).is_none() {
            continue;
        }
        paste_clipboard(ctx.world_mut(), &clipboard, destination.0, ignore_air);
        last_copy = Some(destination);
    }

    match selection_update {
        SelectionUpdate::Keep => {}
        SelectionUpdate::Shift => {
            if let Some(last_copy) = last_copy {
                update_selection(ctx, Some(selection), last_copy)?;
            }
        }
        SelectionUpdate::Expand => {
            let expanded =
                calculate_expanded_selection(first_pos, second_pos, offset * count, false);
            update_selection(ctx, Some(selection), expanded)?;
        }
    }

    let message = if clipped {
        "Your selection was stacked (clipped to plot bounds)."
    } else {
        "Your selection was stacked."
    };
    ctx.worldedit_message(&format!("{message} ({:?})", start_time.elapsed()))
}

fn check_stack_volume(first_pos: BlockPos, second_pos: BlockPos, count: i32) -> CommandResult<()> {
    let volume = i64::from(calculate_selection_volume(first_pos, second_pos)) * i64::from(count);
    if volume > MAX_STACK_VOLUME {
        return Err(CommandError::runtime(format!(
            "Stacking {count} copies would change {volume} blocks, \
             but the limit is {MAX_STACK_VOLUME}."
        )));
    }
    Ok(())
}
