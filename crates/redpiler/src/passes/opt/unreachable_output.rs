//! # [`UnreachableOutput`]
//!
//! This pass uses [`ss_sets`] to find links that can be removed because the
//! output ss of a node is never higher than the weight of the link.

use crate::analysis::ss_sets::ss_sets;
use crate::graph::{Direction, Graph, NodeIdx};
use crate::passes::Pass;

pub struct UnreachableOutput;

impl Pass for UnreachableOutput {
    fn run(&self, graph: &mut Graph) {
        let sets = ss_sets(graph);

        for i in 0..graph.node_bound() {
            let idx = NodeIdx::new(i);
            if !graph.contains_node(idx) {
                continue;
            }
            let max_output = sets.of(idx).max();

            // Now we can go through all the outgoing nodes and remove the ones with a weight that
            // is too high.
            let mut outgoing = graph.neighbors(idx, Direction::Outgoing).detach();
            while let Some((edge_idx, _)) = outgoing.next(graph) {
                if graph[edge_idx].weight >= max_output {
                    graph.remove_edge(edge_idx);
                }
            }
        }
    }

    fn status_message(&self) -> &'static str {
        "Pruning unreachable outputs"
    }

    fn name(&self) -> &'static str {
        "unreachable-output"
    }
}
