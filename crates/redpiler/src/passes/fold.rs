use super::Pass;
use crate::analysis::ss_sets::{ss_sets, InputSets, SSSets};
use crate::behavior::Pending;
use crate::graph::{Direction, Graph, Node, NodeIdx, NodeKind};
use tracing::trace;

#[derive(Debug)]
pub struct Fold;

impl Pass for Fold {
    fn name(&self) -> &'static str {
        "fold"
    }

    fn run(&self, graph: &mut Graph) {
        let sets = ss_sets(graph);
        let folded: Vec<NodeIdx> = graph
            .node_indices()
            .filter(|&idx| never_changes(graph, &sets, idx))
            .collect();
        for &idx in &folded {
            graph[idx].kind = NodeKind::Constant;
            graph.remove_edges(idx, Direction::Incoming);
        }
        trace!("Folded {} nodes", folded.len());

        let nodes: Vec<NodeIdx> = graph.node_indices().collect();
        for idx in nodes {
            let max_output = sets.of(idx).max();
            let mut outgoing = graph.neighbors(idx, Direction::Outgoing).detach();
            while let Some((edge_idx, _)) = outgoing.next(graph) {
                if graph[edge_idx].weight >= max_output {
                    graph.remove_edge(edge_idx);
                }
            }
        }
    }
}

fn never_changes(graph: &Graph, sets: &SSSets, idx: NodeIdx) -> bool {
    let node = &graph[idx];
    if node.is_input()
        || node.pending_tick.is_some()
        || node.kind == NodeKind::Constant
        || node.kind.traits().reads.is_none()
    {
        return false;
    }
    let inputs = sets.inputs(graph, idx);
    inputs.never_update(node) || sets.of(idx).is_single() && lock_never_changes(node, &inputs)
}

fn lock_never_changes(node: &Node, inputs: &InputSets) -> bool {
    inputs.each().all(|inputs| {
        let updated = node.kind.update(node.state, inputs, Pending::None);
        updated.state.repeater_locked == node.state.repeater_locked
    })
}
