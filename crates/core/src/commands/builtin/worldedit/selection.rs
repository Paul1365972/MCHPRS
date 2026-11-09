use crate::commands::argument_parser::{BlockPositionParser, DirectionParser, IntegerParser};
use crate::commands::context::ExecutionContext;
use crate::commands::define::{opt, Command};
use crate::commands::error::{CommandError, CommandResult};
use crate::commands::registry::CommandRegistry;
use crate::commands::value::Direction;
use crate::plot::PLOT_BLOCK_HEIGHT;
use crate::worldedit::{calculate_expanded_selection, calculate_selection_volume};
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;

pub(super) fn register(registry: &mut CommandRegistry) {
    let mut pos = Command::new("/pos");
    let (first_syntax, first) = pos.argument("pos1", BlockPositionParser);
    let (second_syntax, second) = pos.argument("pos2", BlockPositionParser);
    pos.syntax(opt([first_syntax, opt([second_syntax])]));
    registry.register(pos.executes(move |ctx| {
        let pos1 = match ctx.get(first) {
            Some(coordinates) => ctx.block_position(coordinates)?,
            None => ctx.player()?.pos.block_pos(),
        };
        let pos2 = ctx
            .get(second)
            .map(|coordinates| ctx.block_position(coordinates))
            .transpose()?;

        let player = ctx.player_mut()?;
        player.worldedit_set_first_position(pos1);
        if let Some(pos2) = pos2 {
            player.worldedit_set_second_position(pos2);
        }
        Ok(())
    }));

    let mut pos1 = Command::new("/pos1").alias("/1");
    let coordinates = pos1.optional("coordinates", BlockPositionParser);
    registry.register(pos1.executes(move |ctx| {
        let pos = match ctx.get(coordinates) {
            Some(coordinates) => ctx.block_position(coordinates)?,
            None => ctx.player()?.pos.block_pos(),
        };
        set_pos(ctx, Corner::First, pos)
    }));

    let mut pos2 = Command::new("/pos2").alias("/2");
    let coordinates = pos2.optional("coordinates", BlockPositionParser);
    registry.register(pos2.executes(move |ctx| {
        let pos = match ctx.get(coordinates) {
            Some(coordinates) => ctx.block_position(coordinates)?,
            None => ctx.player()?.pos.block_pos(),
        };
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

    let expand = Command::new("/expand")
        .alias("/e")
        .subcommand(Command::new("vert").executes(|ctx| {
            let (first, second) = ctx.selection()?;
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
                    .ok_or_else(CommandError::destination_outside_plot)?;
            let total = calculate_selection_volume(new_first, new_second)
                - calculate_selection_volume(first, second);
            ctx.worldedit_message(&format!("Region expanded {total} block(s)."))
        }));
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
    let reverse_amount = command.optional("reverseAmount", IntegerParser::new(0, 1000));
    let direction = command.optional("direction", DirectionParser);
    registry.register(command.executes(move |ctx| {
        let amount = ctx.get(amount).unwrap_or(1);
        let reverse_amount = ctx.get(reverse_amount);
        let direction = ctx.get(direction).unwrap_or(Direction::Me);
        exec_resize(ctx, contract, amount, reverse_amount, direction)
    }));
}

fn exec_resize(
    ctx: &mut ExecutionContext<'_>,
    contract: bool,
    amount: i32,
    reverse_amount: Option<i32>,
    direction: Direction,
) -> CommandResult<()> {
    let (first, second) = ctx.selection()?;
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
    let plot = ctx.plot().world.region();
    if Region::new(new.0, new.1).intersection(plot).is_none() {
        return Ok(None);
    }
    let clamp = |pos: BlockPos| pos.max(plot.min()).min(plot.max());
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
