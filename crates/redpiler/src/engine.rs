use crate::circuit::NodeId;
use crate::graph::{NodeState, PendingTick};
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::BlockPos;
use serde::{Deserialize, Serialize};
use std::error::Error;
use std::fmt;

pub type EngineError = Box<dyn Error + Send + Sync>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Input {
    Set(SignalStrength),
    Toggle,
    Press,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Drive {
    node: NodeId,
    input: Input,
}

impl Drive {
    pub(crate) fn new(node: NodeId, input: Input) -> Drive {
        Drive { node, input }
    }

    pub fn node(self) -> NodeId {
        self.node
    }

    pub fn input(self) -> Input {
        self.input
    }
}

#[derive(Debug)]
pub enum DriveError {
    NoNode(BlockPos),
    NotAnInput(NodeId),
    Unsupported { node: NodeId, input: Input },
    Engine(EngineError),
}

impl fmt::Display for DriveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DriveError::NoNode(pos) => write!(f, "no node at {pos}"),
            DriveError::NotAnInput(node) => write!(f, "node {} is not an input", node.index()),
            DriveError::Unsupported { node, input } => {
                write!(f, "node {} can't take {input:?}", node.index())
            }
            DriveError::Engine(error) => error.fmt(f),
        }
    }
}

impl Error for DriveError {}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Changes {
    pub states: Vec<(NodeId, NodeState)>,
    pub notes: Vec<(NodeId, u32)>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Snapshot {
    pub states: Vec<NodeState>,
    /// Ticks due together with the same priority run in this order
    pub pending_ticks: Vec<(NodeId, PendingTick)>,
}

/// Keeps at most one pending tick per node between ticks.
pub trait Engine: fmt::Debug + Send {
    /// The drive must come from the circuit the engine was built from.
    fn drive(&mut self, drive: Drive) -> Result<(), EngineError>;

    fn run(&mut self, ticks: u64) -> Result<(), EngineError>;

    /// Runs at most `limit` ticks until none is pending, and returns how many it ran if it got
    /// there.
    fn settle(&mut self, limit: u64) -> Result<Option<u64>, EngineError>;

    /// Appends the observed nodes whose state changed, and the note blocks played, since the last
    /// call.
    fn changes(&mut self, changes: &mut Changes) -> Result<(), EngineError>;

    /// The node must come from the circuit the engine was built from.
    fn state(&mut self, node: NodeId) -> Result<NodeState, EngineError> {
        Ok(self.snapshot()?.states[node.index()])
    }

    fn snapshot(&mut self) -> Result<Snapshot, EngineError>;

    /// The snapshot must pass [`Circuit::check_snapshot`](crate::circuit::Circuit::check_snapshot).
    /// Afterwards, [`Self::changes`] reports every observed node.
    fn restore(&mut self, snapshot: &Snapshot) -> Result<(), EngineError>;

    fn reset(&mut self) -> Result<(), EngineError>;
}

impl<E: Engine + ?Sized> Engine for Box<E> {
    fn drive(&mut self, drive: Drive) -> Result<(), EngineError> {
        (**self).drive(drive)
    }

    fn run(&mut self, ticks: u64) -> Result<(), EngineError> {
        (**self).run(ticks)
    }

    fn settle(&mut self, limit: u64) -> Result<Option<u64>, EngineError> {
        (**self).settle(limit)
    }

    fn changes(&mut self, changes: &mut Changes) -> Result<(), EngineError> {
        (**self).changes(changes)
    }

    fn state(&mut self, node: NodeId) -> Result<NodeState, EngineError> {
        (**self).state(node)
    }

    fn snapshot(&mut self) -> Result<Snapshot, EngineError> {
        (**self).snapshot()
    }

    fn restore(&mut self, snapshot: &Snapshot) -> Result<(), EngineError> {
        (**self).restore(snapshot)
    }

    fn reset(&mut self) -> Result<(), EngineError> {
        (**self).reset()
    }
}
