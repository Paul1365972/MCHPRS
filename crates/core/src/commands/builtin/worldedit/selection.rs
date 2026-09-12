use crate::commands::argument_parser::{DirectionParser, IntegerParser};
use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandError, CommandResult};
use crate::commands::registry::CommandRegistry;
use crate::commands::value::Direction;
use crate::worldedit::{calculate_expanded_selection, calculate_selection_volume};
use mchprs_blocks::BlockPos;

pub(super) fn register(registry: &mut CommandRegistry) {
    registry.register(Command::new("/pos1").alias("/1").executes(|ctx| {
        let pos = ctx.player()?.pos.block_pos();
        set_pos(ctx, Corner::First, pos)
    }));

    registry.register(Command::new("/pos2").alias("/2").executes(|ctx| {
        let pos = ctx.player()?.pos.block_pos();
        set_pos(ctx, Corner::Second, pos)
    }));

    registry.register(Command::new("/hpos1").alias("/h1").executes(|ctx| {
        let pos = ctx.target_block(300.0)?;
        set_pos(ctx, Corner::First, pos)
    }));

    registry.register(Command::new("/hpos2").alias("/h2").executes(|ctx| {
        let pos = ctx.target_block(300.0)?;
        set_pos(ctx, Corner::Second, pos)
    }));

    registry.register(Command::new("/sel").executes(|ctx| {
        ctx.player_mut()?.worldedit_clear_pos();
        Ok(())
    }));

    let expand = Command::new("/expand").alias("/e");
    register_resize(registry, expand, false);
    register_resize(registry, Command::new("/contract"), true);

    let mut shift = Command::new("/shift");
    let amount = shift.optional("amount", IntegerParser::new(1, 1000));
    let direction = shift.optional("direction", DirectionParser);
    registry.register(shift.executes(move |ctx| {
        let amount = ctx.get(amount).unwrap_or(1);
        let direction = ctx.get(direction).unwrap_or(Direction::Me);
        let direction = direction.resolve(ctx.player()?.get_facing());

        let (first, second) = ctx.selection()?;
        let offset = direction.offset_pos(BlockPos::zero(), amount);
        let shifted = (first + offset, second + offset);
        let selected = update_selection(ctx, Some((first, second)), shifted)?
            .ok_or_else(CommandError::destination_outside_plot)?;
        if selected == shifted {
            ctx.worldedit_message(&format!("Region shifted {amount} block(s)."))
        } else {
            ctx.worldedit_message(&format!(
                "Region shifted {amount} block(s) (clipped to plot bounds)."
            ))
        }
    }));
}

#[derive(Clone, Copy)]
enum Corner {
    First,
    Second,
}

fn set_pos(ctx: &mut ExecutionContext<'_>, corner: Corner, pos: BlockPos) -> CommandResult<()> {
    let player = ctx.player_mut()?;
    match corner {
        Corner::First => player.worldedit_set_first_position(pos),
        Corner::Second => player.worldedit_set_second_position(pos),
    }
    Ok(())
}

fn register_resize(registry: &mut CommandRegistry, mut command: Command, contract: bool) {
    let amount = command.optional("amount", IntegerParser::new(1, 1000));
    let direction = command.optional("direction", DirectionParser);
    registry.register(command.executes(move |ctx| {
        let amount = ctx.get(amount).unwrap_or(1);
        let direction = ctx.get(direction).unwrap_or(Direction::Me);
        exec_resize(ctx, contract, amount, direction)
    }));
}

fn exec_resize(
    ctx: &mut ExecutionContext<'_>,
    contract: bool,
    amount: i32,
    direction: Direction,
) -> CommandResult<()> {
    let (first, second) = ctx.selection()?;
    let direction = direction.resolve(ctx.player()?.get_facing());

    let offset = direction.offset_pos(BlockPos::zero(), amount);
    let (new_first, new_second) = calculate_expanded_selection(first, second, offset, contract);

    let old_volume = calculate_selection_volume(first, second);
    let (new_first, new_second) =
        update_selection(ctx, Some((first, second)), (new_first, new_second))?
            .ok_or_else(CommandError::destination_outside_plot)?;
    let new_volume = calculate_selection_volume(new_first, new_second);

    if contract {
        ctx.worldedit_message(&format!(
            "Region contracted {} block(s).",
            old_volume - new_volume
        ))
    } else {
        ctx.worldedit_message(&format!(
            "Region expanded {} block(s).",
            new_volume - old_volume
        ))
    }
}

pub(super) fn update_selection(
    ctx: &mut ExecutionContext<'_>,
    old: Option<(BlockPos, BlockPos)>,
    new: (BlockPos, BlockPos),
) -> CommandResult<Option<(BlockPos, BlockPos)>> {
    if ctx.clip_region(new.0, new.1).is_none() {
        return Ok(None);
    }
    let (plot_min, plot_max) = ctx.plot().world.get_corners();
    let clamp = |pos: BlockPos| pos.max(plot_min).min(plot_max);
    let new = (clamp(new.0), clamp(new.1));
    let player = ctx.player_mut()?;
    if old.is_none_or(|old| new.0 != old.0) {
        player.worldedit_set_first_position(new.0);
    }
    if old.is_none_or(|old| new.1 != old.1) {
        player.worldedit_set_second_position(new.1);
    }
    Ok(Some(new))
}
