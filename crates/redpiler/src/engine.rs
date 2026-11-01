use crate::circuit::{Circuit, NodeId};
use crate::graph::{NodeState, PendingTick};
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::BlockPos;
use std::error::Error;
use std::fmt;
use std::sync::Arc;

pub type EngineError = Box<dyn Error + Send + Sync>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Input {
    Set(SignalStrength),
    Toggle,
    Press,
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
            DriveError::NoNode(position) => write!(f, "no node at {position}"),
            DriveError::NotAnInput(node) => write!(f, "node {} is not an input", node.index()),
            DriveError::Unsupported { node, input } => {
                write!(f, "node {} can't take {input:?}", node.index())
            }
            DriveError::Engine(error) => error.fmt(f),
        }
    }
}

impl Error for DriveError {}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Changes {
    pub states: Vec<(NodeId, NodeState)>,
    pub notes: Vec<(NodeId, u32)>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Snapshot {
    pub states: Vec<NodeState>,
    /// Ticks due together with the same priority run in this order
    pub pending_ticks: Vec<(NodeId, PendingTick)>,
}

/// Runs a [`Circuit`], keeping at most one pending tick per node between ticks.
pub trait Engine: Send {
    fn circuit(&self) -> &Arc<Circuit>;

    /// The input must pass [`Circuit::check_input`].
    fn drive(&mut self, node: NodeId, input: Input) -> Result<(), EngineError>;

    fn run(&mut self, ticks: u64) -> Result<(), EngineError>;

    fn is_idle(&mut self) -> Result<bool, EngineError>;

    fn settle(&mut self, limit: u64) -> Result<Option<u64>, EngineError> {
        for ticks in 0..limit {
            if self.is_idle()? {
                return Ok(Some(ticks));
            }
            self.run(1)?;
        }
        Ok(self.is_idle()?.then_some(limit))
    }

    /// The observed nodes whose state changed, and the note blocks played, since the last call.
    fn changes(&mut self) -> Result<Changes, EngineError>;

    fn snapshot(&mut self) -> Result<Snapshot, EngineError>;

    /// Rejects what [`Circuit::check_snapshot`] rejects. Afterwards, [`Self::changes`] reports
    /// every observed node.
    fn restore(&mut self, snapshot: &Snapshot) -> Result<(), EngineError>;
}

impl<E: Engine + ?Sized> Engine for Box<E> {
    fn circuit(&self) -> &Arc<Circuit> {
        (**self).circuit()
    }

    fn drive(&mut self, node: NodeId, input: Input) -> Result<(), EngineError> {
        (**self).drive(node, input)
    }

    fn run(&mut self, ticks: u64) -> Result<(), EngineError> {
        (**self).run(ticks)
    }

    fn is_idle(&mut self) -> Result<bool, EngineError> {
        (**self).is_idle()
    }

    fn settle(&mut self, limit: u64) -> Result<Option<u64>, EngineError> {
        (**self).settle(limit)
    }

    fn changes(&mut self) -> Result<Changes, EngineError> {
        (**self).changes()
    }

    fn snapshot(&mut self) -> Result<Snapshot, EngineError> {
        (**self).snapshot()
    }

    fn restore(&mut self, snapshot: &Snapshot) -> Result<(), EngineError> {
        (**self).restore(snapshot)
    }
}
