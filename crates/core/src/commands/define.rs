use super::argument::{AllFlags, ArgParser, OneOf, Optional, Parameter, Required, Switch};
use super::context::ExecutionContext;
use super::error::CommandResult;
use super::executor::Executor;
use super::graph::{Node, NodeId, NodeKind, RegistrationError};
use super::help::CommandHelp;
use super::registry::CommandRegistry;
use super::syntax::Syntax;
use mchprs_syntax::flag::Flag;
use std::sync::Arc;

pub(super) struct Command {
    name: String,
    aliases: Vec<String>,
    subcommands: Vec<Command>,
    syntax: Vec<Syntax>,
    flags: Vec<Flag>,
    slots: usize,
    executor: Option<Executor>,
}

impl Command {
    pub(super) fn new(name: &str) -> Self {
        Self {
            name: name.to_owned(),
            aliases: Vec::new(),
            subcommands: Vec::new(),
            syntax: Vec::new(),
            flags: Vec::new(),
            slots: 0,
            executor: None,
        }
    }

    pub(super) fn alias(mut self, alias: &str) -> Self {
        self.aliases.push(alias.to_owned());
        self
    }

    pub(super) fn subcommand(mut self, command: Command) -> Self {
        self.subcommands.push(command);
        self
    }

    pub(super) fn required<P: ArgParser>(&mut self, name: &str, parser: P) -> Required<P::Output> {
        let (syntax, slot) = self.parameter(name, parser);
        self.syntax.push(syntax);
        Required::new(slot)
    }

    pub(super) fn optional<P: ArgParser>(&mut self, name: &str, parser: P) -> Optional<P::Output> {
        let (syntax, slot) = self.parameter(name, parser);
        self.syntax.push(Syntax::Optional(vec![syntax]));
        Optional::new(slot)
    }

    pub(super) fn argument<P: ArgParser>(
        &mut self,
        name: &str,
        parser: P,
    ) -> (Syntax, Optional<P::Output>) {
        let (syntax, slot) = self.parameter(name, parser);
        (syntax, Optional::new(slot))
    }

    pub(super) fn syntax(&mut self, syntax: Syntax) {
        self.syntax.push(syntax);
    }

    pub(super) fn flag(&mut self, flag: Flag) -> Switch {
        self.flags.push(flag);
        Switch(flag)
    }

    pub(super) fn one_of<T: Copy, const N: usize>(
        &mut self,
        choices: [(Flag, T); N],
    ) -> OneOf<T, N> {
        let group = choices.first().map_or("", |(flag, _)| flag.name());
        for (flag, _) in &choices {
            self.flags.push(flag.excluding_others_of(group));
        }
        OneOf(choices)
    }

    pub(super) fn flags_from(&mut self, flags: &[Flag]) -> AllFlags {
        self.flags.extend_from_slice(flags);
        AllFlags
    }

    pub(super) fn executes(
        mut self,
        executor: impl Fn(&mut ExecutionContext<'_>) -> CommandResult<()> + Send + Sync + 'static,
    ) -> Self {
        self.executor = Some(Arc::new(executor));
        self
    }

    fn parameter<P: ArgParser>(&mut self, name: &str, parser: P) -> (Syntax, usize) {
        let slot = self.slots;
        self.slots += 1;
        (
            Syntax::Argument(name.to_owned(), Parameter::value(slot, parser)),
            slot,
        )
    }

    pub(super) fn register(
        mut self,
        registry: &mut CommandRegistry,
        parent: NodeId,
    ) -> Result<NodeId, RegistrationError> {
        if registry.graph.child(parent, &self.name).is_some() {
            return Err(RegistrationError::Conflict(self.name));
        }
        let id = registry
            .graph
            .insert(parent, Node::new(NodeKind::Literal(self.name), None))?;
        for command in self.subcommands {
            command.register(registry, id)?;
        }

        if let Some(executor) = self.executor {
            if !self.flags.is_empty() {
                let flags = Parameter::Flags(self.flags.clone().into());
                self.syntax
                    .push(opt([Syntax::Argument("flags".to_owned(), flags)]));
            }
            let syntax = Syntax::Sequence(self.syntax);
            registry.help.insert(
                id,
                CommandHelp {
                    usage: syntax.to_string(),
                    flags: self.flags,
                },
            );

            for terminal in syntax.register(&mut registry.graph, id)? {
                registry.graph.set_executor(terminal, executor.clone());
            }
        } else {
            assert!(
                self.syntax.is_empty() && self.flags.is_empty(),
                "command {} has arguments but no executor",
                registry.graph.node(id).name()
            );
        }

        for alias in self.aliases {
            if registry.graph.child(parent, &alias).is_some() {
                return Err(RegistrationError::Conflict(alias));
            }
            let executor = registry.graph.node(id).executor.clone();
            let alias_id = registry
                .graph
                .insert(parent, Node::new(NodeKind::Literal(alias), executor))?;
            registry.graph.redirect(alias_id, id)?;
        }

        Ok(id)
    }
}

pub(super) fn opt<const N: usize>(items: [Syntax; N]) -> Syntax {
    Syntax::Optional(items.into())
}

pub(super) fn alt<const N: usize>(alternatives: [Syntax; N]) -> Syntax {
    assert!(N > 0, "empty alternatives");
    Syntax::Alternatives(alternatives.into())
}
