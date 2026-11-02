use crate::graph::Graph;
use crate::passes::Pass;

pub struct ClampWeights;

impl Pass for ClampWeights {
    fn run(&self, graph: &mut Graph) {
        graph.retain_edges(|g, edge| g[edge].weight < 15);
    }

    fn status_message(&self) -> &'static str {
        "Clamping weights"
    }

    fn name(&self) -> &'static str {
        "clamp-weights"
    }
}
