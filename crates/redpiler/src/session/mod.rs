mod worker;

pub use worker::Worker;

use crate::engine::{Changes, Drive, EngineError, Snapshot};
use mchprs_redstone::pace::{Advanced, Goal, TickRate};
use std::fmt;
use std::time::Duration;

/// Dropping a session must not wait for it.
pub trait Session: Send {
    /// A session starts paused.
    fn set_tick_rate(&mut self, rate: TickRate);

    /// An advance runs until it reaches its goal or the next call ends it.
    fn advance(&mut self, goal: Option<Goal>);

    fn drive(&mut self, drive: Drive);

    /// Never waits, so there may be no update yet, or one with changes from different ticks. After
    /// an error the session is gone.
    fn poll(&mut self) -> Result<Option<Update>, SessionError>;

    /// Waits for a tick boundary, and the changes carry every node's state. After
    /// [`SessionError::Unresponsive`] the session still runs.
    fn flush(&mut self, timeout: Duration) -> Result<Update, SessionError>;

    /// Waits for a tick boundary. With `reset`, the snapshot is the state the engine was built in.
    fn stop(self: Box<Self>, reset: bool, timeout: Duration) -> Result<Stopped, SessionError>;
}

/// What happened since the previous update, except `behind` and `advancing`, which are the
/// current status.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Update {
    pub changes: Changes,
    pub ticks: u64,
    pub ticking: Duration,
    pub behind: bool,
    pub advancing: u64,
    pub advanced: Vec<Advanced>,
    pub driven: bool,
}

impl Update {
    fn append(&mut self, later: Update) {
        self.changes.states.extend(later.changes.states);
        self.changes.notes.extend(later.changes.notes);
        self.ticks += later.ticks;
        self.ticking += later.ticking;
        self.behind = later.behind;
        self.advancing = later.advancing;
        self.advanced.extend(later.advanced);
        self.driven |= later.driven;
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Stopped {
    pub update: Update,
    pub snapshot: Snapshot,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SessionContext {
    pub name: String,
}

#[derive(Debug)]
pub enum SessionError {
    Failed(EngineError),
    Panicked(String),
    Unresponsive,
}

impl fmt::Display for SessionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SessionError::Failed(error) => error.fmt(f),
            SessionError::Panicked(message) => write!(f, "it panicked: {message}"),
            SessionError::Unresponsive => write!(f, "it stopped responding"),
        }
    }
}

impl std::error::Error for SessionError {}
