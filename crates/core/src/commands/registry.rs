use crate::commands::{define::Command, help::CommandHelp, node::CommandGraph};
use mchprs_commands::graph::NodeId;
use mchprs_commands::reader::Span;
use std::collections::BTreeMap;
use tracing::warn;

pub struct CommandRegistry {
    pub(super) graph: CommandGraph,
    pub(super) aliases: BTreeMap<String, String>,
    pub(super) help: BTreeMap<NodeId, CommandHelp>,
}

pub(super) struct Expansion {
    pub text: String,
    mappings: Vec<(Span, Span, bool)>,
}

impl Expansion {
    pub fn original_cursor(&self, cursor: usize) -> usize {
        let mapping = self
            .mappings
            .iter()
            .find(|(expanded, _, _)| cursor >= expanded.start && cursor < expanded.end)
            .or_else(|| {
                self.mappings
                    .iter()
                    .rev()
                    .find(|(expanded, _, _)| cursor == expanded.end)
            });
        mapping.map_or(0, |(expanded, original, copied)| {
            if *copied {
                original.start + (cursor - expanded.start).min(original.end - original.start)
            } else {
                original.start
            }
        })
    }

    pub fn original_span(&self, span: Span) -> Option<Span> {
        self.mappings
            .iter()
            .find_map(|(expanded, original, copied)| {
                (*copied && span.start >= expanded.start && span.end <= expanded.end).then(|| {
                    Span::new(
                        original.start + span.start - expanded.start,
                        original.start + span.end - expanded.start,
                    )
                })
            })
    }

    pub fn expanded_cursor(&self, cursor: usize) -> Option<usize> {
        self.mappings
            .iter()
            .find_map(|(expanded, original, copied)| {
                (*copied && cursor >= original.start && cursor <= original.end)
                    .then(|| expanded.start + cursor - original.start)
            })
    }
}

impl CommandRegistry {
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

    pub(super) fn new() -> Self {
        Self {
            graph: CommandGraph::default(),
            aliases: BTreeMap::new(),
            help: BTreeMap::new(),
        }
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
        let name_end = input.find(' ').unwrap_or(input.len());
        let Some(template) = self.aliases.get(&input[..name_end]) else {
            return Expansion {
                text: input.to_owned(),
                mappings: vec![(Span::new(0, input.len()), Span::new(0, input.len()), true)],
            };
        };
        let rest_start = (name_end + 1).min(input.len());
        let rest = &input[rest_start..];
        let mut expansion = Expansion {
            text: String::new(),
            mappings: Vec::new(),
        };
        let mut append = |text: &str, original: Span, copied: bool| {
            let start = expansion.text.len();
            expansion.text.push_str(text);
            expansion
                .mappings
                .push((Span::new(start, expansion.text.len()), original, copied));
        };
        if template.contains("{}") {
            let parts: Vec<_> = template.split("{}").collect();
            for (index, part) in parts.iter().enumerate() {
                let part = if rest.is_empty() && index > 0 {
                    part.strip_prefix(' ').unwrap_or(part)
                } else {
                    part
                };
                append(part, Span::new(0, name_end), false);
                if index + 1 < parts.len() {
                    append(rest, Span::new(rest_start, input.len()), true);
                }
            }
        } else {
            append(template, Span::new(0, name_end), false);
            if name_end < input.len() {
                append(" ", Span::new(name_end, rest_start), false);
                append(rest, Span::new(rest_start, input.len()), true);
            }
        }
        if name_end == input.len() {
            let end = expansion.text.trim_end().len();
            expansion.text.truncate(end);
            for (span, _, _) in &mut expansion.mappings {
                span.start = span.start.min(end);
                span.end = span.end.min(end);
            }
        }
        expansion
    }
}
