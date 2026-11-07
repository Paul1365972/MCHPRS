use crate::circuit::Circuit;
use crate::extract::{extract, ExtractError, ExtractOptions};
use crate::graph::Graph;
use crate::passes::Pass;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::region::Region;
use mchprs_redstone::tick_queue::TickEntry;
use std::fmt;
use std::sync::atomic::{AtomicBool, Ordering};
use tracing::trace;

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
pub struct StageReport {
    /// `extracted`, or the name of the pass
    pub name: &'static str,
    pub nodes: usize,
    pub links: usize,
}

impl StageReport {
    pub fn new(name: &'static str, graph: &Graph) -> StageReport {
        StageReport {
            name,
            nodes: graph.node_count(),
            links: graph.edge_count(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Compiled {
    pub circuit: Circuit,
    pub stages: Vec<StageReport>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Cancelled;

type Inspector<'a> = &'a mut dyn FnMut(&StageReport, &Graph);

pub struct BuildContext<'a> {
    progress: Option<&'a mut dyn FnMut(Stage)>,
    inspect: Option<Inspector<'a>>,
    cancelled: &'a AtomicBool,
}

impl<'a> BuildContext<'a> {
    pub fn new(progress: &'a mut dyn FnMut(Stage), cancelled: &'a AtomicBool) -> BuildContext<'a> {
        BuildContext {
            progress: Some(progress),
            inspect: None,
            cancelled,
        }
    }

    pub fn silent() -> BuildContext<'a> {
        BuildContext {
            progress: None,
            inspect: None,
            cancelled: &NEVER_CANCELLED,
        }
    }

    /// Shows `inspect` each stage's report and the graph it leaves as soon as the stage ends.
    pub fn inspecting(self, inspect: Inspector<'a>) -> BuildContext<'a> {
        BuildContext {
            inspect: Some(inspect),
            ..self
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

    fn finished(&mut self, stage: &StageReport, graph: &Graph) {
        if let Some(inspect) = &mut self.inspect {
            inspect(stage, graph);
        }
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
) -> Result<Compiled, CompileError> {
    let (graph, stages) = compile_graph(world, region, ticks, options, passes, context)?;
    Ok(Compiled {
        circuit: Circuit::new(graph)
            .expect("extraction and the passes keep the circuit invariants"),
        stages,
    })
}

pub fn compile_graph(
    world: &impl BlockView,
    region: Region,
    ticks: &[TickEntry],
    options: &ExtractOptions,
    passes: &[&dyn Pass],
    context: &mut BuildContext<'_>,
) -> Result<(Graph, Vec<StageReport>), CompileError> {
    let mut graph = extract(world, region, ticks, options)?;
    let extracted = StageReport::new("extracted", &graph);
    context.finished(&extracted, &graph);
    context.report(Stage::Extracted {
        nodes: graph.node_count(),
    })?;
    let mut stages = vec![extracted];
    stages.extend(optimize(&mut graph, passes, context)?);
    Ok((graph, stages))
}

pub fn optimize(
    graph: &mut Graph,
    passes: &[&dyn Pass],
    context: &mut BuildContext<'_>,
) -> Result<Vec<StageReport>, Cancelled> {
    let mut stages = Vec::with_capacity(passes.len());
    for (completed, pass) in passes.iter().enumerate() {
        context.report(Stage::Optimizing {
            pass: pass.status_message(),
            completed,
            total: passes.len(),
        })?;
        trace!("Running pass: {}", pass.name());
        pass.run(graph);
        trace!(
            "{} nodes and {} links remain",
            graph.node_count(),
            graph.edge_count()
        );
        let stage = StageReport::new(pass.name(), graph);
        context.finished(&stage, graph);
        stages.push(stage);
    }
    Ok(stages)
}
