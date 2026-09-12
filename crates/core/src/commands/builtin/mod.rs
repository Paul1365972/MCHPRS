mod core;
mod plot;
mod redpiler;
mod worldedit;

use super::argument_parser::GreedyParser;
use super::context::ExecutionContext;
use super::define::Command;
use super::error::CommandResult;
use super::graph::NodeKind;
use super::help::generate_help;
use super::registry::CommandRegistry;
use crate::commands::COMMAND_REGISTRY;

pub(super) fn register_commands(registry: &mut CommandRegistry) {
    core::register_commands(registry);
    plot::register_commands(registry);
    redpiler::register_commands(registry);
    worldedit::register_commands(registry);
    let mut help = Command::new("help").alias("/help");
    let command = help.optional("command", GreedyParser);
    registry.register(help.executes(move |ctx| {
        let command = ctx.get(command).unwrap_or_default();
        exec_help(ctx, &command)
    }));
}

fn exec_help(ctx: &mut ExecutionContext<'_>, command: &str) -> CommandResult<()> {
    let registry = &*COMMAND_REGISTRY;
    if command.is_empty() {
        let mut names: Vec<_> = registry.commands().map(|name| format!("/{name}")).collect();
        names.sort();
        ctx.reply("Run /help <command> for more information.")?;
        return ctx.reply(&names.join(", "));
    }
    let alternate = command
        .strip_prefix('/')
        .map(str::to_owned)
        .unwrap_or_else(|| format!("/{command}"));
    for candidate in [command, alternate.as_str()] {
        let parsed = registry.graph.parse(candidate);
        let literals: Vec<_> = parsed
            .context
            .nodes
            .iter()
            .copied()
            .take_while(|&id| matches!(registry.graph.node(id).kind, NodeKind::Literal(_)))
            .collect();
        if !literals.is_empty() {
            let help = generate_help(registry, &literals);
            if help.usages.is_empty() {
                continue;
            }
            for usage in help.usages {
                ctx.reply(&format!("Usage: {usage}"))?;
            }
            for flag in help.flags {
                ctx.reply(&format!("{flag}: {}", flag.description()))?;
            }
            return Ok(());
        }
    }
    ctx.reply(&format!("Command not found: {command}"))
}
