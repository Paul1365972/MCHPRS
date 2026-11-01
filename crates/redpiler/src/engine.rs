use crate::compile_graph::{NodeState, NodeType, SignalStrength};
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NodeOutput {
    pub power: SignalStrength,
    pub repeater_locked: bool,
}

impl NodeOutput {
    pub fn is_powered(self) -> bool {
        !self.power.is_zero()
    }
}

impl From<NodeState> for NodeOutput {
    fn from(state: NodeState) -> Self {
        NodeOutput {
            power: state.power,
            repeater_locked: state.repeater_locked,
        }
    }
}

#[derive(Debug, Default)]
pub struct NodeChanges {
    pub outputs: Vec<(NodeId, NodeOutput)>,
    pub notes: Vec<NodeId>,
}

#[derive(Debug, Default)]
pub struct EngineSnapshot {
    pub changes: NodeChanges,
    pub pending_ticks: Vec<PendingTick>,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Reporting {
    #[default]
    All,
    InputsAndOutputsWhileRunning,
    InputsAndOutputs,
}

impl Reporting {
    pub fn reports_every_block_while_running(self) -> bool {
        self == Reporting::All
    }

    pub fn snapshots_every_block(self) -> bool {
        self != Reporting::InputsAndOutputs
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Input {
    Interact,
    PressurePlate { powered: bool },
}

impl Input {
    pub fn is_accepted_by(self, node_type: &NodeType) -> bool {
        match self {
            Input::Interact => matches!(node_type, NodeType::Lever | NodeType::Button),
            Input::PressurePlate { .. } => matches!(node_type, NodeType::PressurePlate),
        }
    }
}

pub trait Engine: Send {
    fn run_ticks(&mut self, max_ticks: u64, deadline: Instant) -> Result<u64, EngineError>;
    fn input(&mut self, node: NodeId, input: Input) -> Result<(), EngineError>;
    fn take_changes(&mut self) -> Result<NodeChanges, EngineError>;
    fn snapshot(&mut self) -> Result<EngineSnapshot, EngineError>;
}
