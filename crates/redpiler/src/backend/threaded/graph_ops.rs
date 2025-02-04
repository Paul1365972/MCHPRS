use std::collections::BTreeMap;
use std::fmt::{self, Display};

use crate::compile_graph::{CompileGraph, LinkType, NodeIdx, NodeType};
use itertools::Itertools;
use petgraph::unionfind::UnionFind;
use petgraph::visit::{EdgeRef, IntoEdgeReferences, NodeIndexable};
use petgraph::Direction;
use rustc_hash::{FxHashMap, FxHashSet};

// Partition the graph via Connected-component labeling
pub fn partition_graph_ccl(graph: &CompileGraph) -> ConnectedComponentLabels {
    let mut separator_nodes = vec![false; graph.node_bound()];
    for index in 0..graph.node_bound() {
        let idx = NodeIdx::new(index);
        if graph.contains_node(idx) {
            separator_nodes[index] = is_node_ccl_separator(graph, idx);
        }
    }

    let mut vertex_set = UnionFind::new(graph.node_bound());
    for edge in graph.edge_references() {
        let (u, v) = (edge.source(), edge.target());
        if graph[edge.source()].ty == NodeType::Constant {
            continue;
        }
        if !separator_nodes[u.index()] && !separator_nodes[v.index()] {
            vertex_set.union(u.index(), v.index());
        }
    }

    let mut partitions: FxHashMap<NodeIdx, usize> = FxHashMap::default();
    let mut separators = FxHashSet::default();
    for node in graph.node_indices() {
        if separator_nodes[node.index()] {
            separators.insert(node);
        } else {
            let root = vertex_set.find(node.index());
            partitions.insert(node, root);
        }
    }

    ConnectedComponentLabels {
        partitions,
        separators,
    }
}

fn make_graph_separators_unique(graph: &mut CompileGraph, labels: &ConnectedComponentLabels) {
    for &idx in &labels.separators {
        let node = &graph[idx];
        let connections = graph
            .neighbors_directed(idx, Direction::Incoming)
            .cartesian_product(
                graph
                    .neighbors_directed(idx, Direction::Outgoing)
                    .collect_vec(),
            )
            .collect_vec();
    }
}

fn minimize_graph_separators(
    graph: &CompileGraph,
    mut labels: ConnectedComponentLabels,
) -> ConnectedComponentLabels {
    let partition_sizes = labels.partitions.values().copied().counts();

    labels
}

fn is_node_ccl_separator(graph: &CompileGraph, idx: NodeIdx) -> bool {
    let node = &graph[idx];
    node.is_removable()
        && matches!(
            node.ty,
            NodeType::Repeater { .. }
                | NodeType::Torch
                | NodeType::Comparator {
                    far_input: None,
                    ..
                }
        )
        && graph
            .edges_directed(idx, Direction::Incoming)
            .exactly_one()
            .is_ok_and(|edge| edge.weight().ty == LinkType::Default)
}

fn is_line(graph: &CompileGraph, idx: NodeIdx) -> bool {
    graph
        .edges_directed(idx, Direction::Incoming)
        .exactly_one()
        .is_ok_and(|edge| edge.weight().ty == LinkType::Default)
        && graph
            .edges_directed(idx, Direction::Outgoing)
            .exactly_one()
            .is_ok_and(|edge| edge.weight().ty == LinkType::Default)
}

pub struct ConnectedComponentLabels {
    partitions: FxHashMap<NodeIdx, usize>,
    separators: FxHashSet<NodeIdx>,
}

impl fmt::Display for ConnectedComponentLabels {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let clusters = self
            .partitions
            .values()
            .copied()
            .counts()
            .into_values()
            .counts();

        writeln!(f, "Cluster sizes:")?;
        for (size, count) in clusters.iter().sorted() {
            writeln!(f, "  Size {}: {} cluster(s)", size, count)?;
        }

        writeln!(f, "Separators: {}", self.separators.len())
    }
}
