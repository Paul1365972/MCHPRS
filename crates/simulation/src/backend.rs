use crate::layout::Layout;
use crate::ports::Role;
use crate::ticks::SimulationError;
use crate::{Blocks, Simulation};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_redstone::tick_queue::TickEntry;
use mchprs_syntax::flag::{Flag, FlagValues};
use std::sync::atomic::{AtomicBool, Ordering};

pub const STALE: &str = "is stale: its inputs would change it, but it has no tick scheduled";

pub trait Backend: Send + Sync {
    fn name(&self) -> &str;

    fn flags(&self) -> &[Flag];

    fn compile(
        &self,
        source: &Source<'_>,
        flags: &FlagValues,
        context: &mut CompileContext<'_>,
    ) -> Result<(Box<dyn Simulation<Blocks>>, Layout), CompileError>;
}

#[derive(Debug, thiserror::Error)]
pub enum CompileError {
    #[error(transparent)]
    Cancelled(#[from] Cancelled),
    #[error(transparent)]
    Port(#[from] PortError),
    #[error("{0}")]
    Stale(String),
    #[error("{0}")]
    Unsupported(String),
    #[error(transparent)]
    Failed(SimulationError),
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum PortError {
    #[error("no component at declared port {0}")]
    NoComponent(BlockPos),
    #[error("declared input at {0} can't be driven")]
    NotDrivable(BlockPos),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error("build cancelled")]
pub struct Cancelled;

pub struct Source<'a> {
    pub blocks: &'a dyn BlockView,
    pub region: Region,
    pub ticks: &'a [TickEntry],
    pub declared: &'a [(BlockPos, Role)],
    pub observed: &'a [BlockPos],
}

impl<'a> Source<'a> {
    pub fn new(blocks: &'a dyn BlockView, region: Region) -> Source<'a> {
        Source {
            blocks,
            region,
            ticks: &[],
            declared: &[],
            observed: &[],
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompileStep {
    pub name: &'static str,
    pub completed: usize,
    pub total: usize,
}

impl CompileStep {
    pub fn single(name: &'static str) -> CompileStep {
        CompileStep {
            name,
            completed: 0,
            total: 1,
        }
    }
}

pub struct CompileContext<'a> {
    progress: Option<&'a mut dyn FnMut(CompileStep)>,
    cancelled: Option<&'a AtomicBool>,
}

impl<'a> CompileContext<'a> {
    pub fn new(
        progress: &'a mut dyn FnMut(CompileStep),
        cancelled: &'a AtomicBool,
    ) -> CompileContext<'a> {
        CompileContext {
            progress: Some(progress),
            cancelled: Some(cancelled),
        }
    }

    pub fn silent() -> CompileContext<'a> {
        CompileContext {
            progress: None,
            cancelled: None,
        }
    }

    pub fn reborrow(&mut self) -> CompileContext<'_> {
        CompileContext {
            progress: match &mut self.progress {
                Some(progress) => Some(&mut **progress),
                None => None,
            },
            cancelled: self.cancelled,
        }
    }

    pub fn report(&mut self, step: CompileStep) -> Result<(), Cancelled> {
        if let Some(progress) = &mut self.progress {
            progress(step);
        }
        if self.is_cancelled() {
            return Err(Cancelled);
        }
        Ok(())
    }

    pub fn is_cancelled(&self) -> bool {
        self.cancelled
            .is_some_and(|cancelled| cancelled.load(Ordering::Relaxed))
    }
}
