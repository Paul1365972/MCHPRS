use super::Pass;
use crate::graph::{Direction, Graph, NodeKind};

#[derive(Debug)]
pub struct SortLinks;

impl Pass for SortLinks {
    fn name(&self) -> &'static str {
        "sort-links"
    }

    fn run(&self, graph: &mut Graph) {
        let nodes: Vec<_> = graph.node_indices().collect();
        for node in nodes {
            graph.sort_edges_by_key(node, Direction::Outgoing, |graph, edge| {
                let target = edge.target();
                (kind_order(graph[target].kind), edge.weight().weight, target)
            });
        }
    }
}

fn kind_order(kind: NodeKind) -> u8 {
    match kind {
        NodeKind::Repeater { .. } => 0,
        NodeKind::Torch => 1,
        NodeKind::Comparator { .. } => 2,
        NodeKind::Lamp => 3,
        NodeKind::Button { .. } => 4,
        NodeKind::Lever => 5,
        NodeKind::PressurePlate => 6,
        NodeKind::Trapdoor => 7,
        NodeKind::Wire => 8,
        NodeKind::Constant => 9,
        NodeKind::NoteBlock { .. } => 10,
    }
}
