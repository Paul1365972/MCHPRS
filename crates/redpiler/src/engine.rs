use crate::compile_graph::NodeState;
use crate::netlist::NodeId;
use mchprs_world::TickPriority;
use std::time::Instant;

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
    fn run_ticks(&mut self, max_ticks: u64, deadline: Instant) -> u64;
    fn input(&mut self, node: NodeId, input: Input);
    fn take_changes(&mut self) -> NodeChanges;
    fn snapshot(&mut self) -> (NodeChanges, Vec<PendingTick>);
}
