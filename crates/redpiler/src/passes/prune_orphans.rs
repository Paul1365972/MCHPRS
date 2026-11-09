use super::Pass;
use crate::graph::{Direction, Graph};
use itertools::Itertools;

#[derive(Debug)]
pub struct PruneOrphans;

impl Pass for PruneOrphans {
    fn name(&self) -> &'static str {
        "prune-orphans"
    }

    fn run(&self, graph: &mut Graph) {
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

        graph.retain_nodes(|idx| visited[idx.index()]);
    }
}
