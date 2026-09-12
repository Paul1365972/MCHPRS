use crate::{
    commands::{
        argument::ArgumentType, context::ExecutionContext, define::Command, error::CommandResult,
        registry::CommandRegistry, value::Direction,
    },
    worldedit::{calculate_expanded_selection, calculate_selection_volume},
};
use mchprs_blocks::BlockPos;

pub(super) fn register(registry: &mut CommandRegistry) {
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

    registry.register(
        Command::new("/pos1")
            .alias("/1")
            .permission("worldedit.selection.pos")
            .require_plot_ownership()
            .executes(|ctx| exec_pos(ctx, true)),
    );

    registry.register(
        Command::new("/pos2")
            .alias("/2")
            .permission("worldedit.selection.pos")
            .require_plot_ownership()
            .executes(|ctx| exec_pos(ctx, false)),
    );

    registry.register(
        Command::new("/hpos1")
            .alias("/h1")
            .permission("worldedit.selection.hpos")
            .require_plot_ownership()
            .executes(|ctx| {
                let pos = ctx.target_block(300.0)?;
                set_pos(ctx, true, pos)
            }),
    );

    registry.register(
        Command::new("/hpos2")
            .alias("/h2")
            .permission("worldedit.selection.hpos")
            .require_plot_ownership()
            .executes(|ctx| {
                let pos = ctx.target_block(300.0)?;
                set_pos(ctx, false, pos)
            }),
    );

    registry.register(
        Command::new("/sel")
            .require_plot_ownership()
            .executes(|ctx| {
                ctx.player_mut()?.worldedit_clear_pos();
                Ok(())
            }),
    );

    fn exec_resize(ctx: &mut ExecutionContext<'_>, contract: bool) -> CommandResult<()> {
        let amount: i32 = ctx.arg_or("amount", 1)?;
        let direction = ctx.arg_or("direction", Direction::Me)?;

        let (first, second) = ctx.get_selection()?;
        let direction = direction.resolve(ctx.player()?.get_facing());

        let offset = direction.offset_pos(BlockPos::zero(), amount);
        let (new_first, new_second) = calculate_expanded_selection(first, second, offset, contract);

        let old_volume = calculate_selection_volume(first, second);
        let new_volume = calculate_selection_volume(new_first, new_second);
        update_selection(ctx, (first, second), (new_first, new_second))?;

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

    registry.register(
        Command::new("/expand")
            .alias("/e")
            .permission("worldedit.selection.expand")
            .require_plot_ownership()
            .optional("amount", ArgumentType::integer(1, 1000))
            .optional("direction", ArgumentType::Direction)
            .executes(|ctx| exec_resize(ctx, false)),
    );

    registry.register(
        Command::new("/contract")
            .permission("worldedit.selection.contract")
            .require_plot_ownership()
            .optional("amount", ArgumentType::integer(1, 1000))
            .optional("direction", ArgumentType::Direction)
            .executes(|ctx| exec_resize(ctx, true)),
    );

    registry.register(
        Command::new("/shift")
            .permission("worldedit.selection.shift")
            .require_plot_ownership()
            .optional("amount", ArgumentType::integer(1, 1000))
            .optional("direction", ArgumentType::Direction)
            .executes(|ctx| {
                let amount: i32 = ctx.arg_or("amount", 1)?;
                let direction = ctx.arg_or("direction", Direction::Me)?;
                let direction = direction.resolve(ctx.player()?.get_facing());

                let (first, second) = ctx.get_selection()?;
                let offset = direction.offset_pos(BlockPos::zero(), amount);

                let player = ctx.player_mut()?;
                player.worldedit_set_first_pos(first + offset);
                player.worldedit_set_second_pos(second + offset);
                ctx.worldedit_message(&format!("Region shifted {} block(s).", amount))
            }),
    );
}

pub(super) fn update_selection(
    ctx: &mut ExecutionContext<'_>,
    old: (BlockPos, BlockPos),
    new: (BlockPos, BlockPos),
) -> CommandResult<()> {
    let player = ctx.player_mut()?;
    if new.0 != old.0 {
        player.worldedit_set_first_pos(new.0);
    }
    if new.1 != old.1 {
        player.worldedit_set_second_pos(new.1);
    }
    Ok(())
}
