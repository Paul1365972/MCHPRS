use super::argument::ArgumentType;
use super::context::ExecutionContext;
use super::error::CommandResult;
use mchprs_commands::graph::{Graph, Node};
use std::sync::Arc;

pub(super) type Executor =
    Arc<dyn Fn(&mut ExecutionContext<'_>) -> CommandResult<()> + Send + Sync>;

pub(super) type CommandGraph = Graph<ArgumentType, Executor>;
pub(super) type RegisteredNode = Node<ArgumentType, Executor>;
