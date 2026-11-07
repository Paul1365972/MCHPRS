//! # [`Fold`]
//!
//! Turns every node whose state can never change into a constant that keeps its state. By
//! [`ss_sets`], that is a node that is never updated, or one whose output can take only one value
//! and, for a repeater, whose lock can't change either. Folded nodes lose their input links.

use crate::analysis::ss_sets::{ss_sets, SSSet, SSSets};
use crate::graph::{Direction, Graph, Node, NodeIdx, NodeKind};
use crate::passes::Pass;
use tracing::trace;

pub struct Fold;

impl Pass for Fold {
    fn run(&self, graph: &mut Graph) {
        let sets = ss_sets(graph);
        let folded: Vec<NodeIdx> = graph
            .node_indices()
            .filter(|&idx| never_changes(graph, &sets, idx))
            .collect();
        for &idx in &folded {
            graph[idx].kind = NodeKind::Constant;
            let mut incoming = graph.neighbors(idx, Direction::Incoming).detach();
            while let Some(edge) = incoming.next_edge(graph) {
                graph.remove_edge(edge);
            }
        }
        trace!("Folded {} nodes", folded.len());
    }

    fn status_message(&self) -> &'static str {
        "Folding nodes that never change"
    }

    fn name(&self) -> &'static str {
        "fold"
    }
}

fn never_changes(graph: &Graph, sets: &SSSets, idx: NodeIdx) -> bool {
    let node = &graph[idx];
    if node.is_input()
        || node.pending_tick.is_some()
        || node.kind == NodeKind::Constant
        || !node.kind.is_link_target()
    {
        return false;
    }
    let inputs = sets.inputs(graph, idx);
    inputs.never_update(node) || sets.of(idx).is_single() && lock_never_changes(node, inputs.side)
}

fn lock_never_changes(node: &Node, side: SSSet) -> bool {
    if !matches!(node.kind, NodeKind::Repeater { .. }) {
        return true;
    }
    if node.state.repeater_locked {
        !side.contains(0)
    } else {
        side.max() == 0
    }
}
