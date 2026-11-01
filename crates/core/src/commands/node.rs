use crate::commands::{argument::ArgumentType, context::ExecutionContext, error::CommandResult};
use mchprs_commands::graph::{Graph, Node};
use std::sync::Arc;

pub type Executor = Arc<dyn Fn(&mut ExecutionContext<'_>) -> CommandResult<()> + Send + Sync>;

pub type CommandGraph = Graph<ArgumentType, Executor>;
pub type RegisteredNode = Node<ArgumentType, Executor>;
