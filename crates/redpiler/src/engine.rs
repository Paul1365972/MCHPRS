use crate::compile_graph::{NodeState, NodeType, SignalStrength};
use crate::netlist::NodeId;
use mchprs_world::{SendRate, Step, TickPriority, TickRate, TickSchedule};
use std::error::Error;
use std::time::{Duration, Instant};

pub type EngineError = Box<dyn Error + Send + Sync>;

const ADVANCE_SLICE: Duration = Duration::from_secs(1);

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Resume {
    Now,
    After(Duration),
    OnRequest,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Progress {
    pub ticks_completed: u64,
    pub ticks_owed: u64,
    pub resume: Resume,
}

/// An engine paces its own ticks by the tick rate; `run` makes progress until the deadline.
pub trait Engine: Send {
    fn set_tick_rate(&mut self, rate: TickRate) -> Result<(), EngineError>;
    fn set_send_rate(&mut self, rate: SendRate) -> Result<(), EngineError>;
    fn run(&mut self, deadline: Instant) -> Result<Progress, EngineError>;
    fn input(&mut self, node: NodeId, input: Input) -> Result<(), EngineError>;
    fn take_changes(&mut self) -> Result<NodeChanges, EngineError>;
    fn snapshot(&mut self) -> Result<EngineSnapshot, EngineError>;

    fn advance(&mut self, ticks: u64) -> Result<(), EngineError> {
        self.set_tick_rate(TickRate::Ticks(ticks))?;
        while self.run(Instant::now() + ADVANCE_SLICE)?.ticks_owed != 0 {}
        Ok(())
    }
}

/// An engine that runs up to a number of ticks when asked; `HostPaced` gives it a tick rate.
pub trait SteppedEngine: Send {
    fn run_ticks(&mut self, max_ticks: u64, deadline: Instant) -> Result<u64, EngineError>;
    fn input(&mut self, node: NodeId, input: Input) -> Result<(), EngineError>;
    fn take_changes(&mut self) -> Result<NodeChanges, EngineError>;
    fn snapshot(&mut self) -> Result<EngineSnapshot, EngineError>;
}

pub struct HostPaced<E> {
    engine: E,
    schedule: TickSchedule,
}

impl<E: SteppedEngine> HostPaced<E> {
    pub fn new(engine: E) -> HostPaced<E> {
        HostPaced {
            engine,
            schedule: TickSchedule::new(Instant::now()),
        }
    }
}

impl<E: SteppedEngine> Engine for HostPaced<E> {
    fn set_tick_rate(&mut self, rate: TickRate) -> Result<(), EngineError> {
        self.schedule.set_rate(rate, Instant::now());
        Ok(())
    }

    fn set_send_rate(&mut self, _rate: SendRate) -> Result<(), EngineError> {
        Ok(())
    }

    fn run(&mut self, deadline: Instant) -> Result<Progress, EngineError> {
        let (ticks_completed, resume) = match self.schedule.next(Instant::now()) {
            Step::Run(due) => {
                let completed = self.engine.run_ticks(due, deadline)?;
                self.schedule.complete(completed);
                (completed, Resume::Now)
            }
            Step::WaitFor(wait) => (0, Resume::After(wait)),
            Step::Idle => (0, Resume::OnRequest),
        };
        Ok(Progress {
            ticks_completed,
            ticks_owed: self.schedule.ticks_owed(),
            resume,
        })
    }

    fn input(&mut self, node: NodeId, input: Input) -> Result<(), EngineError> {
        self.engine.input(node, input)
    }

    fn take_changes(&mut self) -> Result<NodeChanges, EngineError> {
        self.engine.take_changes()
    }

    fn snapshot(&mut self) -> Result<EngineSnapshot, EngineError> {
        self.engine.snapshot()
    }
}
