use super::define::Command;
use super::graph::{Graph, Node, NodeId, NodeKind, RegistrationError};
use super::help::CommandHelp;
use mchprs_syntax::expect::Expect;
use mchprs_syntax::flag::{self, Flag, FlagValues};
use mchprs_syntax::reader::{Reader, SyntaxError};
use std::collections::BTreeMap;

pub struct CommandRegistry {
    pub(super) graph: Graph,
    pub(super) presets: BTreeMap<NodeId, FlagValues>,
    pub(super) help: BTreeMap<NodeId, CommandHelp>,
}

impl CommandRegistry {
    pub(super) fn new() -> Self {
        Self {
            graph: Graph::default(),
            presets: BTreeMap::new(),
            help: BTreeMap::new(),
        }
    }

    pub(super) fn commands(&self) -> impl Iterator<Item = &str> {
        let root = self.graph.node(self.graph.root());
        root.children().iter().map(|&id| self.graph.node(id).name())
    }

    pub(super) fn register(&mut self, command: Command) {
        command
            .register(self, self.graph.root())
            .unwrap_or_else(|error| panic!("Invalid command definition: {error}"));
    }

    pub(super) fn alias(&mut self, name: &str, template: &str) {
        let installed = if self.graph.child(self.graph.root(), name).is_some() {
            Err(AliasError::Taken(name.to_owned()))
        } else {
            self.resolve_alias(name, template)
                .and_then(|alias| self.install_alias(alias))
        };
        installed.unwrap_or_else(|error| panic!("Invalid alias `{name}`: {error}"));
    }

    pub(super) fn configured_alias(&self, name: &str, template: &str) -> Result<Alias, AliasError> {
        let (Some(name), Some(template)) = (name.strip_prefix('/'), template.strip_prefix('/'))
        else {
            return Err(AliasError::MissingSlash);
        };
        self.resolve_alias(name, template)
    }

    pub(super) fn install_alias(&mut self, alias: Alias) -> Result<(), AliasError> {
        let root = self.graph.root();
        if let Some(shadowed) = self.graph.detach(root, &alias.name) {
            self.presets.remove(&shadowed);
        }
        let executor = self.graph.node(alias.target).executor.clone();
        let id = self
            .graph
            .insert(root, Node::new(NodeKind::Literal(alias.name), executor))?;
        self.graph.redirect(id, alias.target)?;
        self.presets.insert(id, alias.preset);
        Ok(())
    }

    fn resolve_alias(&self, name: &str, template: &str) -> Result<Alias, AliasError> {
        let root = self.graph.root();
        let mut reader = Reader::new(template);
        let mut target = root;
        while reader.can_read() && reader.peek() != Some('-') {
            let word = reader.take_while(|ch| ch != ' ');
            let child = self
                .graph
                .child(target, word)
                .filter(|&child| matches!(self.graph.node(child).kind, NodeKind::Literal(_)))
                .ok_or_else(|| AliasError::UnknownCommand(word.to_owned()))?;
            if self.presets.contains_key(&child) {
                return Err(AliasError::AliasOfAlias);
            }
            target = self.graph.node(child).redirect().unwrap_or(child);
            reader.take_while(|ch| ch == ' ');
        }
        if target == root {
            return Err(AliasError::UnknownCommand(template.to_owned()));
        }
        let commands = match self.help.get(&target) {
            Some(help) => vec![help],
            None => self
                .commands_below(target)
                .into_iter()
                .map(|(_, help)| help)
                .collect(),
        };
        let accepted: Vec<Flag> = commands
            .iter()
            .flat_map(|help| help.flags.clone())
            .collect();
        let preset =
            flag::parse(&mut reader, &accepted, &mut Expect::executing()).map_err(|error| {
                AliasError::Flags {
                    template: template.to_owned(),
                    error,
                }
            })?;
        if let Some(flag) = accepted
            .iter()
            .filter(|flag| preset.contains(flag))
            .find(|flag| !commands.iter().all(|help| help.flags.contains(flag)))
        {
            return Err(AliasError::FlagNotEverywhere(flag.name()));
        }
        Ok(Alias {
            name: name.to_owned(),
            target,
            preset,
        })
    }

    pub(super) fn commands_below(&self, target: NodeId) -> Vec<(Vec<&str>, &CommandHelp)> {
        let target = self.graph.node(target).redirect().unwrap_or(target);
        let mut commands: Vec<_> = self
            .help
            .get(&target)
            .map(|help| (Vec::new(), help))
            .into_iter()
            .collect();
        for &child in self.graph.node(target).children() {
            let node = self.graph.node(child);
            if matches!(node.kind, NodeKind::Literal(_)) && node.redirect().is_none() {
                commands.extend(
                    self.commands_below(child)
                        .into_iter()
                        .map(|(mut path, help)| {
                            path.insert(0, node.name());
                            (path, help)
                        }),
                );
            }
        }
        commands
    }

    pub(super) fn refused_preset(&self, path: &[NodeId]) -> Option<&'static str> {
        let command = path.iter().rev().find_map(|id| self.help.get(id))?;
        let preset = self.preset(path);
        let refused = preset.flags().find(|flag| !command.flags.contains(flag));
        refused.map(Flag::name)
    }

    pub(super) fn preset(&self, path: &[NodeId]) -> FlagValues {
        let mut preset = FlagValues::default();
        for id in path {
            if let Some(flags) = self.presets.get(id) {
                preset.extend(flags.clone());
            }
        }
        preset
    }
}

pub(super) struct Alias {
    name: String,
    target: NodeId,
    preset: FlagValues,
}

#[derive(Debug, thiserror::Error)]
pub(super) enum AliasError {
    #[error("both sides start with /, like \"/rpx\" = \"/rp c -io\"")]
    MissingSlash,
    #[error("`{0}` is already a command")]
    Taken(String),
    #[error("`{0}` is not a command")]
    UnknownCommand(String),
    #[error("an alias can't run another alias")]
    AliasOfAlias,
    #[error("`--{0}` doesn't apply to every command the alias can run")]
    FlagNotEverywhere(&'static str),
    #[error("{}", .error.contextual(.template))]
    Flags {
        template: String,
        error: SyntaxError,
    },
    #[error(transparent)]
    Registration(#[from] RegistrationError),
}
