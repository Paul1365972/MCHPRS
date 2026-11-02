use crate::circuit::Circuit;
use crate::extract::{extract, ExtractError, ExtractOptions};
use crate::graph::Graph;
use crate::passes::{run_passes, Pass};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::region::Region;
use mchprs_redstone::tick_queue::TickEntry;
use std::fmt;
use std::ops::ControlFlow;

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
    Building,
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

pub fn compile(
    world: &impl BlockView,
    region: Region,
    ticks: &[TickEntry],
    options: &ExtractOptions,
    passes: &[&dyn Pass],
    progress: &mut dyn FnMut(Stage) -> ControlFlow<()>,
) -> Result<Circuit, CompileError> {
    let mut graph = extract(world, region, ticks, options)?;
    let nodes = graph.node_count();
    if progress(Stage::Extracted { nodes }).is_break() {
        return Err(CompileError::Cancelled);
    }
    if optimize(&mut graph, passes, progress).is_break() || progress(Stage::Building).is_break() {
        return Err(CompileError::Cancelled);
    }
    Ok(Circuit::new(graph).expect("extraction and the passes keep the circuit invariants"))
}

fn optimize(
    graph: &mut Graph,
    passes: &[&dyn Pass],
    progress: &mut dyn FnMut(Stage) -> ControlFlow<()>,
) -> ControlFlow<()> {
    let mut starting = |completed: usize| match passes.get(completed) {
        Some(pass) => progress(Stage::Optimizing {
            pass: pass.status_message(),
            completed,
            total: passes.len(),
        }),
        None => ControlFlow::Continue(()),
    };
    starting(0)?;
    let mut completed = 0;
    run_passes(passes, graph, &mut |_, _| {
        completed += 1;
        starting(completed)
    })
}
