use crate::compile_graph::CompileGraph;
use crate::passes::{AnalysisInfos, Pass};

pub struct ClampWeights;

impl Pass for ClampWeights {
    fn run_pass(&self, graph: &mut CompileGraph, _: &mut AnalysisInfos) {
        graph.retain_edges(|g, edge| g[edge].weight < 15);
    }

    fn status_message(&self) -> &'static str {
        "Clamping weights"
    }

    fn driver_key(&self) -> &'static str {
        "clamp-weights"
    }
}
