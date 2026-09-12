use crate::commands::{argument::ArgumentType, context::ExecutionContext, error::CommandResult};
use crate::player::Player;
use mchprs_commands::graph::{Graph, Node};
use std::sync::Arc;

pub type Executor = Arc<dyn Fn(&mut ExecutionContext<'_>) -> CommandResult<()> + Send + Sync>;

pub type CommandGraph = Graph<ArgumentType, Executor, Policy>;

pub type RegisteredNode = Node<ArgumentType, Executor, Policy>;

#[derive(Clone, Default, PartialEq, Eq)]
pub struct Policy {
    pub permissions: Vec<String>,
    pub requires_plot_ownership: bool,
}

impl Policy {
    pub fn allows(&self, player: &Player) -> bool {
        self.permissions
            .iter()
            .all(|permission| player.has_permission(permission))
    }
}
