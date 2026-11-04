mod alias;
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
pub mod registry;
pub mod schematic;
pub mod suggestions;
pub mod value;

use self::registry::CommandRegistry;
use crate::config::ServerConfig;

pub fn command_registry(config: &ServerConfig) -> CommandRegistry {
    let mut registry = CommandRegistry::new();
    builtin::register_commands(&mut registry, config);
    for (alias, replacement) in &config.command_aliases {
        registry.add_custom_alias(alias.trim(), replacement.trim());
    }
    registry.remove_invalid_aliases();
    registry
}
