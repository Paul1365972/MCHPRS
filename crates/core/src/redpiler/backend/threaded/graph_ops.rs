use std::collections::BTreeMap;
use std::fmt::{self, Display};

use crate::redpiler::compile_graph::{CompileGraph, LinkType, NodeIdx, NodeType};
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

fn minimize_graph_separators(
    graph: &CompileGraph,
    mut labels: ConnectedComponentLabels,
) -> ConnectedComponentLabels {
    loop {
        let mut affected = None;
        for node in labels.separators.iter() {
            let u = graph
                .neighbors_directed(*node, Direction::Incoming)
                .exactly_one()
                .unwrap();
            let v = graph
                .neighbors_directed(*node, Direction::Outgoing)
                .exactly_one()
                .unwrap();
            match (labels.partitions.get(&u), labels.partitions.get(&v)) {
                (Some(partition), None) | (None, Some(partition)) => {
                    affected = Some((*node, *partition));
                }
                (Some(partition_u), Some(partition_v)) if partition_u == partition_v => {
                    affected = Some((*node, *partition_u));
                }
                _ => {}
            }
        }
        let Some((node, partition)) = affected else {
            break;
        };
        labels.separators.remove(&node);
        labels.partitions.insert(node, partition);
    }

    labels
}

fn is_node_ccl_separator(graph: &CompileGraph, idx: NodeIdx) -> bool {
    let node = &graph[idx];
    node.is_removable()
        && matches!(
            node.ty,
            NodeType::Repeater {
                facing_diode: false,
                ..
            }
        )
        && is_line(graph, idx)
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
        let sizes = clusters.iter().sorted();

        writeln!(f, "Cluster sizes:")?;
        for (size, count) in sizes {
            writeln!(f, "  Size {}: {} cluster(s)", size, count)?;
        }

        writeln!(f, "Separators: {}", self.separators.len())
    }
}
