use crate::graph::{
    Direction, Graph, LinkKind as GraphLinkKind, NodeIdx, NodeKind as GraphNodeKind,
};
use itertools::Itertools;
use mchprs_blocks::blocks::ComparatorMode as CComparatorMode;
use redpiler_graph::{
    serialize, BlockPos, ComparatorMode, Link, LinkType, Node, NodeState, NodeType,
};
use rustc_hash::FxHashMap;

fn convert_node(graph: &Graph, node_idx: NodeIdx, nodes_map: &FxHashMap<NodeIdx, usize>) -> Node {
    let node = &graph[node_idx];

    let mut inputs = Vec::new();
    for edge in graph.edges(node_idx, Direction::Incoming) {
        let idx = nodes_map[&edge.source()];
        let link = edge.weight();
        inputs.push(Link {
            ty: match link.kind {
                GraphLinkKind::Default => LinkType::Default,
                GraphLinkKind::Side => LinkType::Side,
            },
            weight: link.weight,
            to: idx,
        });
    }

    let updates = graph
        .neighbors(node_idx, Direction::Outgoing)
        .map(|idx| nodes_map[&idx])
        .collect();

    let diode_prioritized = match node.kind {
        GraphNodeKind::Repeater { prioritized, .. }
        | GraphNodeKind::Comparator { prioritized, .. } => prioritized,
        _ => false,
    };

    let comparator_far_input = match node.kind {
        GraphNodeKind::Comparator { far_input, .. } => far_input,
        _ => None,
    };

    Node {
        ty: match node.kind {
            GraphNodeKind::Repeater { delay, .. } => NodeType::Repeater(delay),
            GraphNodeKind::Torch => NodeType::Torch,
            GraphNodeKind::Comparator { mode, .. } => NodeType::Comparator(match mode {
                CComparatorMode::Compare => ComparatorMode::Compare,
                CComparatorMode::Subtract => ComparatorMode::Subtract,
            }),
            GraphNodeKind::Lamp => NodeType::Lamp,
            GraphNodeKind::Button => NodeType::Button,
            GraphNodeKind::Lever => NodeType::Lever,
            GraphNodeKind::PressurePlate => NodeType::PressurePlate,
            GraphNodeKind::Trapdoor => NodeType::Trapdoor,
            GraphNodeKind::Wire => NodeType::Wire,
            GraphNodeKind::Constant => NodeType::Constant,
            GraphNodeKind::NoteBlock { .. } => NodeType::NoteBlock,
        },
        block: node
            .block
            .iter()
            .copied()
            .map(|(pos, id)| {
                (
                    BlockPos {
                        x: pos.x,
                        y: pos.y,
                        z: pos.z,
                    },
                    id,
                )
            })
            .collect(),
        state: NodeState {
            power: node.state.power,
            repeater_locked: node.state.repeater_locked,
        },
        comparator_far_input,
        diode_prioritized,
        inputs,
        updates,
    }
}

pub fn export_graph(graph: &Graph) -> Vec<u8> {
    let mut nodes_map = FxHashMap::with_capacity_and_hasher(graph.node_count(), Default::default());
    for node in graph.node_indices() {
        nodes_map.insert(node, nodes_map.len());
    }

    let nodes = graph
        .node_indices()
        .map(|idx| convert_node(graph, idx, &nodes_map))
        .collect_vec();

    serialize(nodes.as_slice()).expect("the graph is serializable")
}
