mod core;
mod worldedit;

use crate::commands::{
    argument::ArgumentType, context::ExecutionContext, define::Command, error::CommandResult,
    help::generate_help, registry::CommandRegistry,
};
use crate::config::Backends;
use mchprs_commands::graph::NodeKind;

pub fn register_commands(registry: &mut CommandRegistry, backends: Backends) {
    core::register_commands(registry, backends);
    worldedit::register_commands(registry);
    registry.register(
        Command::new("help")
            .optional("command", ArgumentType::GreedyString)
            .executes(exec_help),
    );
    registry.register(
        Command::new("/help")
            .optional("command", ArgumentType::GreedyString)
            .executes(exec_help),
    );
}

fn exec_help(ctx: &mut ExecutionContext<'_>) -> CommandResult<()> {
    let command: String = ctx.arg_or("command", String::new())?;
    let registry = &ctx.shared().commands;
    if command.is_empty() {
        let mut names: Vec<_> = registry
            .visible_commands()
            .map(|(name, _)| format!("/{name}"))
            .collect();
        names.sort();
        ctx.reply("Run /help <command> for more information.")?;
        return ctx.reply(&names.join(", "));
    }
    let alternate = command
        .strip_prefix('/')
        .map(str::to_owned)
        .unwrap_or_else(|| format!("/{command}"));
    for candidate in [command.as_str(), alternate.as_str()] {
        let expanded = registry.expand(candidate);
        let parsed = registry.graph.parse(&expanded.text);
        let path = parsed.context.path();
        let literals: Vec<_> = path
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
