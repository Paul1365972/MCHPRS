//! # [`PruneOrphans`]
//!
//! This pass removes any nodes in the graph that aren't transitively connected to an output
//! redstone component by using Depth-First-Search.
//!
//! Observed nodes count as outputs, and so do nodes with a pending tick.

use crate::graph::{Direction, Graph};
use crate::passes::Pass;
use itertools::Itertools;

pub struct PruneOrphans;

impl Pass for PruneOrphans {
    fn run(&self, graph: &mut Graph) {
        let mut worklist = graph
            .node_indices()
            .filter(|&idx| graph[idx].observed || graph[idx].pending_tick.is_some())
            .collect_vec();

        let mut visited = vec![false; graph.node_bound()];

        // Visit initial nodes
        for &idx in &worklist {
            visited[idx.index()] = true;
        }

        while let Some(idx) = worklist.pop() {
            for incoming in graph.neighbors(idx, Direction::Incoming) {
                if !visited[incoming.index()] {
                    visited[incoming.index()] = true;
                    worklist.push(incoming);
                }
            }
        }

        graph.retain_nodes(|_, idx| visited[idx.index()]);
    }

    fn status_message(&self) -> &'static str {
        "Pruning orphans"
    }

    fn name(&self) -> &'static str {
        "prune-orphans"
    }
}
