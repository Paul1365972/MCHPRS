use mchprs_redpiler::graph::{Graph, NodeIdx};
use mchprs_ril::ast;

pub fn labelled_graph(circuit: &ast::Circuit) -> Graph {
    let mut graph = circuit.graph();
    let nodes: Vec<NodeIdx> = graph.node_indices().collect();
    for (node, component) in nodes.into_iter().zip(&circuit.components) {
        graph[node].names = component.labels.clone();
    }
    graph
}
