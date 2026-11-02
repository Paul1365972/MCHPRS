use crate::commands::argument::ArgumentType;
use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandResult, RuntimeError};
use crate::commands::registry::CommandRegistry;
use crate::commands::value::Direction;
use crate::worldedit::{calculate_expanded_selection, calculate_selection_volume};
use mchprs_blocks::BlockPos;

pub(super) fn register(registry: &mut CommandRegistry) {
    registry.register(
        Command::new("/pos1")
            .alias("/1")
            .executes(|ctx| exec_pos(ctx, true)),
    );

    registry.register(
        Command::new("/pos2")
            .alias("/2")
            .executes(|ctx| exec_pos(ctx, false)),
    );

    registry.register(Command::new("/hpos1").alias("/h1").executes(|ctx| {
        let pos = ctx.target_block(300.0)?;
        set_pos(ctx, true, pos)
    }));

    registry.register(Command::new("/hpos2").alias("/h2").executes(|ctx| {
        let pos = ctx.target_block(300.0)?;
        set_pos(ctx, false, pos)
    }));

    registry.register(Command::new("/sel").executes(|ctx| {
        ctx.player_mut()?.worldedit_clear_pos();
        Ok(())
    }));

    registry.register(
        Command::new("/expand")
            .alias("/e")
            .optional("amount", ArgumentType::integer(1, 1000))
            .optional("direction", ArgumentType::Direction)
            .executes(|ctx| exec_resize(ctx, false)),
    );

    registry.register(
        Command::new("/contract")
            .optional("amount", ArgumentType::integer(1, 1000))
            .optional("direction", ArgumentType::Direction)
            .executes(|ctx| exec_resize(ctx, true)),
    );

    registry.register(
        Command::new("/shift")
            .optional("amount", ArgumentType::integer(1, 1000))
            .optional("direction", ArgumentType::Direction)
            .executes(|ctx| {
                let amount: i32 = ctx.arg_or("amount", 1)?;
                let direction = ctx.arg_or("direction", Direction::Me)?;
                let direction = direction.resolve(ctx.player()?.get_facing());

                let (first, second) = ctx.selection()?;
                let offset = direction.offset_pos(BlockPos::zero(), amount);
                let shifted = (first + offset, second + offset);
                let selected = update_selection(ctx, Some((first, second)), shifted)?
                    .ok_or(RuntimeError::DestinationOutsidePlot)?;
                if selected == shifted {
                    ctx.worldedit_message(&format!("Region shifted {amount} block(s)."))
                } else {
                    ctx.worldedit_message(&format!(
                        "Region shifted {amount} block(s) (clipped to plot bounds)."
                    ))
                }
            }),
    );
}

fn exec_pos(ctx: &mut ExecutionContext<'_>, first: bool) -> CommandResult<()> {
    let pos = ctx.player()?.pos.block_pos();
    set_pos(ctx, first, pos)
}

fn set_pos(ctx: &mut ExecutionContext<'_>, first: bool, pos: BlockPos) -> CommandResult<()> {
    let player = ctx.player_mut()?;
    if first {
        player.worldedit_set_first_pos(pos);
    } else {
        player.worldedit_set_second_pos(pos);
    }
    Ok(())
}

fn exec_resize(ctx: &mut ExecutionContext<'_>, contract: bool) -> CommandResult<()> {
    let amount: i32 = ctx.arg_or("amount", 1)?;
    let direction = ctx.arg_or("direction", Direction::Me)?;

    let (first, second) = ctx.selection()?;
    let direction = direction.resolve(ctx.player()?.get_facing());

    let offset = direction.offset_pos(BlockPos::zero(), amount);
    let (new_first, new_second) = calculate_expanded_selection(first, second, offset, contract);

    let old_volume = calculate_selection_volume(first, second);
    let (new_first, new_second) =
        update_selection(ctx, Some((first, second)), (new_first, new_second))?
            .ok_or(RuntimeError::DestinationOutsidePlot)?;
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
        player.worldedit_set_first_pos(new.0);
    }
    if old.is_none_or(|old| new.1 != old.1) {
        player.worldedit_set_second_pos(new.1);
    }
    Ok(Some(new))
}
