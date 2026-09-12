mod argument;
mod argument_parser;
mod builtin;
mod context;
mod declaration;
mod define;
mod error;
mod executor;
mod graph;
mod help;
mod parse;
pub mod registry;
pub mod schematic;
pub mod suggestions;
mod syntax;
pub mod value;

use crate::config::{ServerConfig, CONFIG};
use std::sync::LazyLock;
pub static COMMAND_REGISTRY: LazyLock<CommandRegistry> =
    LazyLock::new(|| command_registry(&CONFIG));
use itertools::Itertools;
use registry::CommandRegistry;
use tracing::warn;

pub fn command_registry(config: &ServerConfig) -> CommandRegistry {
    let mut registry = CommandRegistry::new();
    builtin::register_commands(&mut registry);
    let aliases: Vec<_> = config
        .command_aliases
        .iter()
        .sorted()
        .map(|(name, template)| {
            (
                name,
                registry.configured_alias(name.trim(), template.trim()),
            )
        })
        .collect();
    for (name, alias) in aliases {
        if let Err(error) = alias.and_then(|alias| registry.install_alias(alias)) {
            warn!("Ignoring alias `{name}` in Config.toml: {error}");
        }
    }
    registry
}
