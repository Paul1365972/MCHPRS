use crate::commands::{
    argument::ArgumentType,
    context::ExecutionContext,
    error::CommandResult,
    help::CommandHelp,
    node::{Executor, RegisteredNode},
    registry::CommandRegistry,
};
use mchprs_commands::flag::Flag;
use mchprs_commands::graph::{NodeId, NodeKind, RegistrationError};
use std::sync::Arc;

pub type Syntax = mchprs_commands::syntax::Syntax<ArgumentType>;

pub const FLAGS_ARGUMENT: &str = "flags";

pub struct Command {
    name: String,
    aliases: Vec<String>,
    subcommands: Vec<Command>,
    syntax: Vec<Syntax>,
    flags: Vec<Flag>,
    executor: Option<Executor>,
}

pub fn arg(name: &str, arg_type: ArgumentType) -> Syntax {
    assert!(
        name != FLAGS_ARGUMENT,
        "{FLAGS_ARGUMENT} is a reserved name"
    );
    Syntax::Argument(name.to_owned(), arg_type)
}

pub fn opt<const N: usize>(items: [Syntax; N]) -> Syntax {
    Syntax::Optional(items.into())
}

pub fn alt<const N: usize>(alternatives: [Syntax; N]) -> Syntax {
    assert!(N > 0, "empty alternatives");
    Syntax::Alternatives(alternatives.into())
}

impl Command {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_owned(),
            aliases: Vec::new(),
            subcommands: Vec::new(),
            syntax: Vec::new(),
            flags: Vec::new(),
            executor: None,
        }
    }

    pub fn alias(mut self, alias: &str) -> Self {
        self.aliases.push(alias.to_owned());
        self
    }

    pub fn subcommand(mut self, command: Command) -> Self {
        self.subcommands.push(command);
        self
    }

    pub fn arg(mut self, name: &str, arg_type: ArgumentType) -> Self {
        self.syntax.push(arg(name, arg_type));
        self
    }

    pub fn optional(mut self, name: &str, arg_type: ArgumentType) -> Self {
        self.syntax.push(opt([arg(name, arg_type)]));
        self
    }

    pub fn syntax(mut self, syntax: Syntax) -> Self {
        self.syntax.push(syntax);
        self
    }

    pub fn flag(
        mut self,
        short: impl Into<Option<char>>,
        name: &'static str,
        description: &'static str,
    ) -> Self {
        self.flags
            .push(Flag::switch(name, short.into(), description));
        self
    }

    pub fn flags(mut self, flags: &[Flag]) -> Self {
        self.flags.extend_from_slice(flags);
        self
    }

    pub fn executes(
        mut self,
        executor: impl Fn(&mut ExecutionContext<'_>) -> CommandResult<()> + Send + Sync + 'static,
    ) -> Self {
        self.executor = Some(Arc::new(executor));
        self
    }

    pub(super) fn register(
        mut self,
        registry: &mut CommandRegistry,
        parent: NodeId,
    ) -> Result<NodeId, RegistrationError> {
        if registry.graph.child(parent, &self.name).is_some() {
            return Err(RegistrationError::Conflict(self.name));
        }
        let id = registry.graph.insert(
            parent,
            RegisteredNode::new(NodeKind::Literal(self.name), None),
        )?;
        for command in self.subcommands {
            command.register(registry, id)?;
        }

        if let Some(executor) = self.executor {
            if !self.flags.is_empty() {
                self.syntax.push(opt([Syntax::Argument(
                    FLAGS_ARGUMENT.to_owned(),
                    ArgumentType::Flags {
                        flags: self.flags.clone(),
                    },
                )]));
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
            let alias_id = registry.graph.insert(
                parent,
                RegisteredNode::new(NodeKind::Literal(alias), executor),
            )?;
            registry.graph.redirect(alias_id, id)?;
        }

        Ok(id)
    }
}
