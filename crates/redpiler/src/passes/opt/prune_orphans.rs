//! # [`PruneOrphans`]
//!
//! Removes every node that is not observed and leads to no observed node or pending tick.

use crate::graph::{Direction, Graph};
use crate::passes::{AnalysisInfos, Pass};
use itertools::Itertools;

pub struct PruneOrphans;

impl Pass for PruneOrphans {
    fn run_pass(&self, graph: &mut Graph, _: &mut AnalysisInfos) {
        let mut worklist = graph
            .node_indices()
            .filter(|&idx| graph[idx].observed || graph[idx].pending_tick.is_some())
            .collect_vec();

        let mut visited = vec![false; graph.node_bound()];

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

    fn driver_key(&self) -> &'static str {
        "prune-orphans"
    }
}
