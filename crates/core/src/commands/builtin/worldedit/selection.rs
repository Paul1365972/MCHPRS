use crate::{
    commands::{
        argument::ArgumentType,
        context::ExecutionContext,
        define::{arg, opt, Command},
        error::{CommandResult, RuntimeError},
        registry::CommandRegistry,
        value::Direction,
    },
    plot::PLOT_BLOCK_HEIGHT,
    worldedit::{calculate_expanded_selection, calculate_selection_volume},
};
use mchprs_blocks::{region::Region, BlockPos};

pub(super) fn register(registry: &mut CommandRegistry) {
    registry.register(
        Command::new("/pos")
            .syntax(opt([
                arg("pos1", ArgumentType::BlockPos),
                opt([arg("pos2", ArgumentType::BlockPos)]),
            ]))
            .executes(|ctx| {
                let pos1 = ctx
                    .block_position("pos1")?
                    .unwrap_or(ctx.player()?.pos.block_pos());
                let pos2 = ctx.block_position("pos2")?;

                let player = ctx.player_mut()?;
                player.worldedit_set_first_pos(pos1);
                if let Some(pos2) = pos2 {
                    player.worldedit_set_second_pos(pos2);
                }
                Ok(())
            }),
    );

    fn exec_pos(ctx: &mut ExecutionContext<'_>, first: bool) -> CommandResult<()> {
        let pos = ctx
            .block_position("coordinates")?
            .unwrap_or(ctx.player()?.pos.block_pos());
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
            .optional("coordinates", ArgumentType::BlockPos)
            .executes(|ctx| exec_pos(ctx, true)),
    );

    registry.register(
        Command::new("/pos2")
            .alias("/2")
            .optional("coordinates", ArgumentType::BlockPos)
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

    registry.register(
        Command::new("/sel")
            .alias(";")
            .alias("/desel")
            .alias("/deselect")
            .executes(|ctx| {
                ctx.player_mut()?.worldedit_clear_pos();
                Ok(())
            }),
    );

    fn exec_resize(ctx: &mut ExecutionContext<'_>, contract: bool) -> CommandResult<()> {
        let amount: i32 = ctx.arg_or("amount", 1)?;
        let reverse_amount = ctx.arg_opt::<i32>("reverseAmount")?;
        let direction = ctx.arg_or("direction", Direction::Me)?;

        let (first, second) = ctx.get_selection()?;
        let direction = direction.resolve(ctx.player()?.get_facing());

        let offset = direction.offset_pos(BlockPos::zero(), amount);
        let (mut new_first, mut new_second) =
            calculate_expanded_selection(first, second, offset, contract);

        if let Some(reverse_amount) = reverse_amount {
            let reverse_offset = direction.offset_pos(BlockPos::zero(), -reverse_amount);
            (new_first, new_second) =
                calculate_expanded_selection(new_first, new_second, reverse_offset, contract);
        }

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

    registry.register(
        Command::new("/expand")
            .alias("/e")
            .subcommand(Command::new("vert").executes(|ctx| {
                let (first, second) = ctx.get_selection()?;
                let (mut new_first, mut new_second) = (first, second);
                let (lower, upper) = if first.y <= second.y {
                    (&mut new_first, &mut new_second)
                } else {
                    (&mut new_second, &mut new_first)
                };
                lower.y = 0;
                upper.y = PLOT_BLOCK_HEIGHT - 1;

                let (new_first, new_second) =
                    update_selection(ctx, Some((first, second)), (new_first, new_second))?
                        .ok_or(RuntimeError::DestinationOutsidePlot)?;
                let total = calculate_selection_volume(new_first, new_second)
                    - calculate_selection_volume(first, second);
                ctx.worldedit_message(&format!("Region expanded {} block(s).", total))
            }))
            .optional("amount", ArgumentType::integer(1, 1000))
            .optional("reverseAmount", ArgumentType::integer(1, 1000))
            .optional("direction", ArgumentType::Direction)
            .executes(|ctx| exec_resize(ctx, false)),
    );

    registry.register(
        Command::new("/contract")
            .optional("amount", ArgumentType::integer(1, 1000))
            .optional("reverseAmount", ArgumentType::integer(1, 1000))
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

                let (first, second) = ctx.get_selection()?;
                let offset = direction.offset_pos(BlockPos::zero(), amount);
                let shifted = (first + offset, second + offset);
                let selected = update_selection(ctx, Some((first, second)), shifted)?
                    .ok_or(RuntimeError::DestinationOutsidePlot)?;
                if selected == shifted {
                    ctx.worldedit_message(&format!("Region shifted {} block(s).", amount))
                } else {
                    ctx.worldedit_message(&format!(
                        "Region shifted {} block(s) (clipped to plot bounds).",
                        amount
                    ))
                }
            }),
    );
}

pub(super) fn update_selection(
    ctx: &mut ExecutionContext<'_>,
    old: Option<(BlockPos, BlockPos)>,
    new: (BlockPos, BlockPos),
) -> CommandResult<Option<(BlockPos, BlockPos)>> {
    let plot = ctx.plot_mut().world.region();
    if Region::new(new.0, new.1).intersection(plot).is_none() {
        return Ok(None);
    }
    let clamp = |pos: BlockPos| pos.max(plot.min()).min(plot.max());
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
