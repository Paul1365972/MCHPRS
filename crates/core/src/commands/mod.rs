mod argument;
mod argument_parser;
mod argument_suggestions;
mod builtin;
mod context;
mod declaration;
mod define;
mod error;
mod executor;
mod help;
mod node;
mod registry;
pub mod schematic;
pub mod suggestions;
pub mod value;

use crate::config::ServerConfig;

pub use registry::CommandRegistry;

pub fn command_registry(config: &ServerConfig) -> Result<CommandRegistry, Vec<String>> {
    let engines = config.redpiler.engines()?;
    let mut registry = CommandRegistry::new();
    builtin::register_commands(&mut registry, &engines);
    for (alias, replacement) in &config.command_aliases {
        registry.add_custom_alias(alias.trim(), replacement.trim());
    }
    registry.remove_invalid_aliases();
    Ok(registry)
}
