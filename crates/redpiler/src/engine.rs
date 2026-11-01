use crate::compile_graph::NodeState;
use crate::netlist::NodeId;
use mchprs_world::TickPriority;
use std::error::Error;
use std::time::Instant;

pub type EngineError = Box<dyn Error + Send + Sync>;

#[derive(Debug, Clone, Copy)]
pub struct PendingTick {
    pub node: NodeId,
    pub delay: u32,
    pub priority: TickPriority,
}

#[derive(Debug, Default)]
pub struct NodeChanges {
    pub states: Vec<(NodeId, NodeState)>,
    pub notes: Vec<NodeId>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Input {
    Interact,
    PressurePlate { powered: bool },
}

pub trait Engine: Send {
    fn run_ticks(&mut self, max_ticks: u64, deadline: Instant) -> Result<u64, EngineError>;
    fn input(&mut self, node: NodeId, input: Input) -> Result<(), EngineError>;
    fn take_changes(&mut self) -> Result<NodeChanges, EngineError>;
    fn snapshot(&mut self) -> Result<(NodeChanges, Vec<PendingTick>), EngineError>;
}
