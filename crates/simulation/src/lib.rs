pub mod backend;
pub mod block_changes;
pub mod component;
pub mod handle;
pub mod input;
pub mod layout;
pub mod ports;
pub mod readback;
pub mod schedule;
pub mod simulated_world;
pub mod stepper;
pub mod ticks;
pub mod watch;
pub mod write_back;

use crate::block_changes::BlockChanges;
use crate::input::DriveError;
use crate::readback::Readback;
use crate::ticks::{Ran, SimulationError};
use crate::watch::{Hit, Watch};
use crate::write_back::WriteBack;
use mchprs_blocks::BlockPos;
use mchprs_redstone::input::{Input, InputError};
use std::fmt::Debug;
use std::time::Duration;

/// What a simulation is addressed by and what it reports.
pub trait Space: 'static {
    type Address: Copy + Debug + PartialEq + Send;
    type Changes: Debug + Clone + Default + PartialEq + Send + Extend<Self::Changes>;
    type State: Debug + Clone + PartialEq + Send;
}

/// A plot, addressed by block position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Blocks;

impl Space for Blocks {
    type Address = BlockPos;
    type Changes = BlockChanges;
    type State = WriteBack;
}

/// A compiled circuit that the caller steps. Ticks run only inside `run` and `settle`, and it keeps
/// no clock, so a build without threads can use it. Speed is never measured here: only the running
/// form shows the speed the game sees.
pub trait Simulation<S: Space>: Send {
    fn capabilities(&self) -> Capabilities;

    /// Ticks run since it was compiled.
    fn ticks(&self) -> u64;

    /// Runs exactly `ticks`, fewer only when a breakpoint fires.
    fn run(&mut self, ticks: u64) -> Result<Ran, SimulationError>;

    fn settle(&mut self, limit: u64) -> Result<Ran, SimulationError>;

    fn drive(&mut self, at: S::Address, input: Input) -> Result<(), DriveError>;

    fn watch(&mut self, watch: Watch<S::Address>) -> Result<(), SimulationError>;

    /// Appends what `view` asks for.
    fn show(&mut self, view: View, changes: &mut S::Changes) -> Result<(), SimulationError>;

    /// The whole state it reached, read without changing anything.
    fn read(&mut self) -> Result<Readback<S::State>, SimulationError>;

    fn stop(self: Box<Self>) -> Result<End<S>, SimulationError>;

    /// Lets it run by itself from here, paused, with its state, pending ticks, watches and tick
    /// count. The backend decides how. Fails where no thread can be spawned.
    fn start(self: Box<Self>) -> Result<Box<dyn Running<S>>, SimulationError>;
}

/// A compiled circuit that runs by itself. It owns its threads and its timing, and decides how
/// many ticks run and when. One owner holds it, on any thread. An `Err` is final: the simulation
/// is gone. Dropping it ends it without waiting. Speed is measured here, never on the stepped
/// form.
pub trait Running<S: Space>: Send {
    /// What this simulation can do. The owner sends only what they allow.
    fn capabilities(&self) -> Capabilities;

    /// Never waits. Commands take effect in the order sent, each at a tick boundary. While paused
    /// no tick runs: a drive followed by a run of n ticks runs exactly n ticks after the input, on
    /// every backend.
    fn send(&mut self, command: Command<S::Address>);

    /// Never waits. Returns the updates that arrived since the last call, merged.
    fn poll(&mut self) -> Result<Option<Update<S>>, SimulationError>;

    /// Waits for the next update. Call it only when one is owed: after `Show`, or while a run with
    /// a goal runs.
    fn wait(&mut self) -> Result<Update<S>, SimulationError>;

    /// Ends the simulation at the next tick boundary.
    fn stop(self: Box<Self>) -> Result<End<S>, SimulationError>;
}

/// What a simulation leaves when it stops.
#[derive(Debug, Clone, PartialEq)]
pub struct End<S: Space> {
    /// What changed and was not shown yet.
    pub changes: S::Changes,
    pub state: Readback<S::State>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Capabilities {
    /// It halts on breakpoints: `Command::Watch` has an effect.
    pub breakpoints: bool,
    /// It can tell when no tick is pending: `Run::UntilSettled` is supported.
    pub settling: bool,
    /// `read` and `stop` return the state it reached. Without it, the world keeps the state from
    /// before the compile.
    pub write_back: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Command<A> {
    /// No tick runs. A simulation starts paused, and pauses itself after `RunFinished` and `Halted`.
    Pause,
    /// How ticks run from now on. Replaces what ran before.
    Run(Run),
    /// While it runs, the input lands at whichever tick boundary the backend reaches next.
    Drive(A, Input),
    Watch(Watch<A>),
    /// Answered by one update, read at one tick boundary.
    Show(View),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Run {
    /// One tick per interval.
    Every(Duration),
    /// As fast as the backend can.
    Unlimited,
    /// This many ticks as fast as the backend can. Then it pauses and reports `RunFinished`.
    Ticks(u64),
    /// As fast as the backend can until no tick is pending. Then it pauses and reports `RunFinished`.
    UntilSettled,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum View {
    /// What is observed and changed since it was last shown.
    Observed,
    /// Everything that changed since it was last shown, as far as the backend can read it.
    All,
}

/// What happened since the previous update. An update arrives in answer to `Show`, and unasked
/// when it carries an event or an input landed.
#[derive(Debug, Clone, PartialEq)]
pub struct Update<S: Space> {
    pub changes: S::Changes,
    /// Ticks run since the simulation was compiled.
    pub ticks: u64,
    pub events: Vec<Event<S::Address>>,
    /// A rate is not reached.
    pub behind: bool,
}

impl<S: Space> Default for Update<S> {
    fn default() -> Update<S> {
        Update {
            changes: S::Changes::default(),
            ticks: 0,
            events: Vec::new(),
            behind: false,
        }
    }
}

impl<S: Space> Update<S> {
    pub fn append(&mut self, mut later: Update<S>) {
        self.changes.extend([later.changes]);
        self.ticks = later.ticks;
        self.events.append(&mut later.events);
        self.behind = later.behind;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event<A> {
    /// A run of `Ticks` or `UntilSettled` met its goal. Every such run ends with exactly one of
    /// `RunFinished` and `RunInterrupted`, in the order the runs were sent.
    RunFinished,
    /// A later command or a breakpoint ended a run of `Ticks` or `UntilSettled` before its goal.
    RunInterrupted,
    /// A breakpoint fired. The simulation is paused.
    Halted(Hit),
    /// The input at this address was not applied.
    Refused(A, InputError),
    /// This update answers a `Show`.
    Shown(View),
}
