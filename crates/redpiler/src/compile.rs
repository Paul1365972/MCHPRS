use crate::circuit::Circuit;
use crate::extract::{extract, ExtractError, ExtractOptions};
use crate::graph::Graph;
use crate::passes::{run_passes, Pass};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::region::Region;
use mchprs_redstone::tick_queue::TickEntry;
use std::fmt;
use std::ops::ControlFlow;
use std::sync::atomic::{AtomicBool, Ordering};

static NEVER_CANCELLED: AtomicBool = AtomicBool::new(false);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stage {
    Extracted {
        nodes: usize,
    },
    Optimizing {
        pass: &'static str,
        completed: usize,
        total: usize,
    },
    Building {
        step: &'static str,
        completed: usize,
        total: usize,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Cancelled;

pub struct BuildContext<'a> {
    progress: Option<&'a mut dyn FnMut(Stage)>,
    cancelled: &'a AtomicBool,
}

impl<'a> BuildContext<'a> {
    pub fn new(progress: &'a mut dyn FnMut(Stage), cancelled: &'a AtomicBool) -> BuildContext<'a> {
        BuildContext {
            progress: Some(progress),
            cancelled,
        }
    }

    pub fn silent() -> BuildContext<'static> {
        BuildContext {
            progress: None,
            cancelled: &NEVER_CANCELLED,
        }
    }

    pub fn report(&mut self, stage: Stage) -> Result<(), Cancelled> {
        if let Some(progress) = &mut self.progress {
            progress(stage);
        }
        if self.is_cancelled() {
            return Err(Cancelled);
        }
        Ok(())
    }

    pub fn is_cancelled(&self) -> bool {
        self.cancelled.load(Ordering::Relaxed)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CompileError {
    Extract(ExtractError),
    Cancelled,
}

impl fmt::Display for CompileError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CompileError::Extract(error) => error.fmt(f),
            CompileError::Cancelled => write!(f, "compile cancelled"),
        }
    }
}

impl std::error::Error for CompileError {}

impl From<ExtractError> for CompileError {
    fn from(error: ExtractError) -> Self {
        CompileError::Extract(error)
    }
}

impl From<Cancelled> for CompileError {
    fn from(_: Cancelled) -> Self {
        CompileError::Cancelled
    }
}

pub fn compile(
    world: &impl BlockView,
    region: Region,
    ticks: &[TickEntry],
    options: &ExtractOptions,
    passes: &[&dyn Pass],
    context: &mut BuildContext<'_>,
) -> Result<Circuit, CompileError> {
    let mut graph = extract(world, region, ticks, options)?;
    context.report(Stage::Extracted {
        nodes: graph.node_count(),
    })?;
    optimize(&mut graph, passes, context)?;
    Ok(Circuit::new(graph).expect("extraction and the passes keep the circuit invariants"))
}

fn optimize(
    graph: &mut Graph,
    passes: &[&dyn Pass],
    context: &mut BuildContext<'_>,
) -> Result<(), Cancelled> {
    let mut starting = |completed: usize| match passes.get(completed) {
        Some(pass) => context.report(Stage::Optimizing {
            pass: pass.status_message(),
            completed,
            total: passes.len(),
        }),
        None => Ok(()),
    };
    starting(0)?;
    let mut completed = 0;
    let finished = run_passes(passes, graph, &mut |_, _| {
        completed += 1;
        match starting(completed) {
            Ok(()) => ControlFlow::Continue(()),
            Err(Cancelled) => ControlFlow::Break(()),
        }
    });
    match finished {
        ControlFlow::Continue(()) => Ok(()),
        ControlFlow::Break(()) => Err(Cancelled),
    }
}
