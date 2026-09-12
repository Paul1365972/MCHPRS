use super::alias::Expansion;
use super::define::Command;
use super::help::CommandHelp;
use super::node::CommandGraph;
use mchprs_commands::graph::NodeId;
use std::collections::BTreeMap;
use tracing::warn;

pub struct CommandRegistry {
    pub(super) graph: CommandGraph,
    pub(super) aliases: BTreeMap<String, String>,
    pub(super) help: BTreeMap<NodeId, CommandHelp>,
}

impl CommandRegistry {
    pub(super) fn new() -> Self {
        Self {
            graph: CommandGraph::default(),
            aliases: BTreeMap::new(),
            help: BTreeMap::new(),
        }
    }

    pub(super) fn visible_commands(&self) -> impl Iterator<Item = (&str, Option<NodeId>)> {
        let commands = self
            .graph
            .node(self.graph.root())
            .children()
            .iter()
            .filter_map(|&id| {
                let node = self.graph.node(id);
                (!self.aliases.contains_key(node.name())).then_some((node.name(), Some(id)))
            });
        let aliases = self.aliases.keys().map(|name| (name.as_str(), None));
        commands.chain(aliases)
    }

    pub(super) fn register(&mut self, command: Command) {
        command
            .register(self, self.graph.root())
            .unwrap_or_else(|error| panic!("Invalid command definition: {error}"));
    }

    pub(super) fn add_custom_alias(&mut self, name: &str, replacement: &str) {
        self.aliases.insert(name.to_owned(), replacement.to_owned());
    }

    pub(super) fn remove_invalid_aliases(&mut self) {
        let invalid: Vec<_> = self
            .aliases
            .iter()
            .filter_map(|(name, replacement)| {
                let reason = self.alias_error(name, replacement)?;
                Some((name.clone(), reason))
            })
            .collect();
        for (name, reason) in invalid {
            warn!("Ignoring alias `{name}` in Config.toml: {reason}");
            self.aliases.remove(&name);
        }
    }

    fn alias_error(&self, name: &str, replacement: &str) -> Option<String> {
        if name.is_empty() || name.chars().any(char::is_whitespace) || name.contains(['{', '}']) {
            return Some("names can't be empty or contain whitespace or braces".to_owned());
        }
        if replacement
            .split_whitespace()
            .any(|token| token.contains(['{', '}']) && token != "{}")
        {
            return Some("braces are only allowed as a separate `{}`".to_owned());
        }
        let target = replacement.split(' ').next().unwrap_or_default();
        if self.graph.child(self.graph.root(), target).is_some() {
            return None;
        }
        Some(if self.aliases.contains_key(target) {
            format!("`{target}` is an alias, not a command")
        } else {
            format!("`{target}` is not a command")
        })
    }

    pub(super) fn expand(&self, input: &str) -> Expansion {
        Expansion::new(input, &self.aliases)
    }
}
