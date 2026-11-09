pub mod backend;
pub mod block_changes;
pub mod component;
pub mod input;
pub mod layout;
pub mod ports;
pub mod readback;
pub mod simulated_world;
pub mod ticks;
pub mod write_back;

use crate::block_changes::BlockChanges;
use crate::input::DriveError;
use crate::readback::Readback;
use crate::ticks::{Ran, SimulationError};
use crate::write_back::WriteBack;
use mchprs_blocks::BlockPos;
use mchprs_redstone::input::Input;
use std::fmt::Debug;

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

pub trait Simulation<S: Space>: Send {
    fn capabilities(&self) -> Capabilities;

    /// Ticks run since it was compiled.
    fn ticks(&self) -> u64;

    fn run(&mut self, ticks: u64) -> Result<Ran, SimulationError>;

    fn settle(&mut self, limit: u64) -> Result<Ran, SimulationError>;

    fn drive(&mut self, at: S::Address, input: Input) -> Result<(), DriveError>;

    /// Appends what `view` asks for.
    fn show(&mut self, view: View, changes: &mut S::Changes) -> Result<(), SimulationError>;

    /// The whole state it reached, read without changing anything.
    fn read(&mut self) -> Result<Readback<S::State>, SimulationError>;

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
    pub settling: bool,
    /// `read` and `stop` return the state it reached. Without it, the world keeps the state from
    /// before the compile.
    pub write_back: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum View {
    /// What is observed and changed since it was last shown.
    Observed,
    /// Everything that changed since it was last shown, as far as the backend can read it.
    All,
}
