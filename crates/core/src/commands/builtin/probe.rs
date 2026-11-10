use crate::commands::argument_parser::{ConditionParser, DirectionParser, MaskParser, WordParser};
use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandError, CommandResult};
use crate::commands::registry::CommandRegistry;
use crate::commands::value::Direction;
use crate::plot::probe::{InvalidWidth, Probe, ProbeError, MAX_PROBE_BITS};
use crate::worldedit::mask::WorldEditMask;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_simulation::component::powered_at;
use mchprs_simulation::watch::Condition;

pub(super) fn register_commands(registry: &mut CommandRegistry) {
    let mut add = Command::new("add");
    let name = add.required("name", WordParser);
    let mask = add.optional("mask", MaskParser);
    let direction = add.optional("direction", DirectionParser);
    let add = add.executes(move |ctx| {
        let (name, mask, direction) = (ctx.get(name), ctx.get(mask), ctx.get(direction));
        exec_add(ctx, name, mask, direction.unwrap_or(Direction::Me))
    });

    let mut remove = Command::new("remove");
    let removed = remove.required("name", WordParser);
    let remove = remove.executes(move |ctx| {
        let name = ctx.get(removed);
        ctx.plot_mut()
            .remove_probe(&name)
            .map_err(|error| probe_error(error, &name))?;
        ctx.reply(&format!("Removed probe {name}."))
    });

    let mut set_break = Command::new("break");
    let break_name = set_break.required("name", WordParser);
    let condition = set_break.required("condition", ConditionParser);
    let set_break = set_break.executes(move |ctx| {
        let (name, condition) = (ctx.get(break_name), ctx.get(condition));
        exec_break(ctx, &name, condition)
    });

    let mut unbreak = Command::new("unbreak");
    let unbreak_name = unbreak.required("name", WordParser);
    let unbreak = unbreak.executes(move |ctx| {
        let name = ctx.get(unbreak_name);
        let probe = ctx.plot().probe(&name).ok_or_else(|| no_probe(&name))?;
        if probe.breakpoint.is_none() {
            return Err(CommandError::runtime(format!("{name} has no breakpoint.")));
        }
        ctx.plot_mut().set_breakpoint(&name, None);
        ctx.reply(&format!("Removed the breakpoint of {name}."))
    });

    registry.register(
        Command::new("probe")
            .subcommand(add)
            .subcommand(remove)
            .subcommand(Command::new("list").executes(exec_list))
            .subcommand(Command::new("clear").executes(|ctx| {
                let count = ctx.plot_mut().clear_probes();
                ctx.reply(&format!("Removed {}.", counted(count, "probe")))
            }))
            .subcommand(set_break)
            .subcommand(unbreak),
    );
}

fn exec_add(
    ctx: &mut ExecutionContext<'_>,
    name: String,
    mask: Option<WorldEditMask>,
    direction: Direction,
) -> CommandResult<()> {
    if !ctx.plot().simulation.is_editable() {
        return Err(CommandError::runtime(
            "A probe can only be added while Redpiler is stopped. Reset Redpiler first.",
        ));
    }
    let facing = direction.resolve(ctx.player()?.get_facing());
    let (first, second) = ctx.selection()?;
    let world = ctx.world();
    let positions: Vec<BlockPos> = world
        .occupied_positions(Region::new(first, second))
        .filter(|&pos| mask.as_ref().is_none_or(|mask| mask.matches(world, pos)))
        .filter(|&pos| powered_at(world, pos).is_some())
        .collect();
    let components = match mask {
        Some(_) => "components matching the mask",
        None => "components",
    };
    let probe = Probe::new(name.clone(), positions, facing).map_err(|error| match error {
        InvalidWidth::Empty => {
            CommandError::runtime(format!("The selection holds no {components}."))
        }
        InvalidWidth::TooWide(width) => CommandError::runtime(format!(
            "A probe reads at most {MAX_PROBE_BITS} components, but the selection holds {width} {components}."
        )),
    })?;
    let value = probe
        .read(world)
        .expect("the selection holds only components");
    let message = format!(
        "Added probe {} with {}: {}.",
        probe.name,
        counted(probe.width(), "bit"),
        probe.format(value)
    );
    ctx.plot_mut()
        .add_probe(probe)
        .map_err(|error| probe_error(error, &name))?;
    ctx.reply(&message)
}

fn exec_list(ctx: &mut ExecutionContext<'_>) -> CommandResult<()> {
    ctx.plot_mut().sync_redpiler();
    let plot = ctx.plot();
    if plot.probes().is_empty() {
        return ctx.reply("There are no probes. Add one with /probe add.");
    }
    let mut lines = Vec::with_capacity(plot.probes().len());
    for probe in plot.probes() {
        let value = match probe.read(&plot.world) {
            Some(value) => probe.format(value),
            None => "broken".to_owned(),
        };
        let mut line = format!("{}: {value} ({}", probe.name, counted(probe.width(), "bit"));
        if let Some(breakpoint) = probe.breakpoint {
            let condition = probe.describe(breakpoint.condition);
            line.push_str(&format!(", halts when {condition}"));
        }
        line.push(')');
        lines.push(line);
    }
    lines.iter().try_for_each(|line| ctx.reply(line))
}

fn exec_break(
    ctx: &mut ExecutionContext<'_>,
    name: &str,
    condition: Condition,
) -> CommandResult<()> {
    let probe = ctx.plot().probe(name).ok_or_else(|| no_probe(name))?;
    let bits = probe.width();
    if let Condition::Becomes(value) | Condition::Leaves(value) = condition
        && value.checked_shr(bits as u32).is_some_and(|rest| rest != 0)
    {
        return Err(CommandError::runtime(format!(
            "{value} doesn't fit into the {} of {name}.",
            counted(bits, "bit")
        )));
    }
    if !ctx.plot().halts_on_breakpoints() {
        return Err(CommandError::runtime(
            "This backend can't halt on breakpoints. Reset Redpiler to use them.",
        ));
    }
    let message = format!("The plot halts when {}.", probe.describe(condition));
    ctx.plot_mut().set_breakpoint(name, Some(condition));
    ctx.reply(&message)
}

fn probe_error(error: ProbeError, name: &str) -> CommandError {
    match error {
        ProbeError::NameTaken => {
            CommandError::runtime(format!("There is a probe named {name} already."))
        }
        ProbeError::NoProbe => no_probe(name),
    }
}

fn counted(count: usize, noun: &str) -> String {
    match count {
        1 => format!("1 {noun}"),
        _ => format!("{count} {noun}s"),
    }
}

fn no_probe(name: &str) -> CommandError {
    CommandError::runtime(format!("There is no probe named {name}."))
}
