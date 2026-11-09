use crate::circuit::{Circuit, InvalidCircuit};
use crate::extract::{ExtractError, ExtractOptions};
use crate::graph::Graph;
use crate::nodes::Nodes;
use crate::passes::Pass;
use mchprs_simulation::backend::{Backend, Cancelled, CompileContext, CompileError, CompileStep};
use mchprs_simulation::Simulation;
use mchprs_syntax::flag::FlagValues;
use tracing::trace;

pub trait GraphBackend: Backend {
    fn extract_options(&self, _flags: &FlagValues) -> ExtractOptions {
        ExtractOptions::default()
    }

    fn passes(&self, _flags: &FlagValues) -> Vec<Box<dyn Pass>> {
        Vec::new()
    }

    fn optimize(
        &self,
        graph: &mut Graph,
        flags: &FlagValues,
        context: &mut GraphContext<'_>,
    ) -> Result<(), CompileError> {
        Ok(context.run_passes(graph, &self.passes(flags))?)
    }

    fn build(
        &self,
        circuit: &Circuit,
        flags: &FlagValues,
        context: &mut GraphContext<'_>,
    ) -> Result<Box<dyn Simulation<Nodes>>, CompileError>;
}

impl From<ExtractError> for CompileError {
    fn from(error: ExtractError) -> CompileError {
        match error {
            ExtractError::Port(error) => CompileError::Port(error),
            error => CompileError::Unsupported(error.to_string()),
        }
    }
}

impl From<InvalidCircuit> for CompileError {
    fn from(invalid: InvalidCircuit) -> CompileError {
        match invalid {
            InvalidCircuit::Stale { .. } => CompileError::Stale(invalid.to_string()),
            invalid => CompileError::Unsupported(invalid.to_string()),
        }
    }
}

type Inspector<'a> = &'a mut dyn FnMut(&'static str, &Graph);

pub struct GraphContext<'a> {
    compile: CompileContext<'a>,
    inspect: Option<Inspector<'a>>,
}

impl<'a> GraphContext<'a> {
    pub fn new(compile: CompileContext<'a>) -> GraphContext<'a> {
        GraphContext {
            compile,
            inspect: None,
        }
    }

    pub fn silent() -> GraphContext<'a> {
        GraphContext::new(CompileContext::silent())
    }

    pub fn inspecting(self, inspect: Inspector<'a>) -> GraphContext<'a> {
        GraphContext {
            inspect: Some(inspect),
            ..self
        }
    }

    pub fn report(&mut self, step: CompileStep) -> Result<(), Cancelled> {
        self.compile.report(step)
    }

    pub fn is_cancelled(&self) -> bool {
        self.compile.is_cancelled()
    }

    pub fn inspect(&mut self, step: &'static str, graph: &Graph) {
        if let Some(inspect) = &mut self.inspect {
            inspect(step, graph);
        }
    }

    pub fn run_passes(
        &mut self,
        graph: &mut Graph,
        passes: &[Box<dyn Pass>],
    ) -> Result<(), Cancelled> {
        for (completed, pass) in passes.iter().enumerate() {
            self.report(CompileStep {
                name: pass.name(),
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
            self.inspect(pass.name(), graph);
        }
        Ok(())
    }
}
