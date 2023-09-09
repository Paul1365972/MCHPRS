//! # [`SccSeperation`]
//!

use itertools::Itertools;
use petgraph::visit::{EdgeRef, IntoEdgeReferences};
use rustc_hash::FxHashMap;

use super::Pass;
use crate::redpiler::compile_graph::{CompileGraph, CompileLink, CompileNode, LinkType, NodeType};
use crate::redpiler::{CompilerInput, CompilerOptions};
use crate::world::World;

pub struct SccSeperation;

impl<W: World> Pass<W> for SccSeperation {
    fn run_pass(&self, graph: &mut CompileGraph, _: &CompilerOptions, _: &CompilerInput<'_, W>) {
        let scc = petgraph::algo::kosaraju_scc(&*graph);

        let mut subgraph_map =
            FxHashMap::with_capacity_and_hasher(graph.node_count(), Default::default());
        for (id, component) in scc.into_iter().enumerate() {
            for idx in component {
                subgraph_map.insert(idx, id);
            }
        }

        let critical_edges = graph
            .edge_references()
            .filter(|edge| subgraph_map[&edge.source()] != subgraph_map[&edge.target()])
            .map(|edge| edge.id())
            .collect_vec();
        for edge_idx in critical_edges {
            let (start_node_idx, end_node_idx) = graph.edge_endpoints(edge_idx).unwrap();
            let state = graph[end_node_idx].state.clone();

            let input_node = graph.add_node(CompileNode {
                ty: NodeType::SubgraphInput(0.into()),
                block: None,
                state: state.clone(),
                facing_diode: false,
                comparator_far_input: None,
                is_input: true,
                is_output: false,
            });
            let output_node = graph.add_node(CompileNode {
                ty: NodeType::SubgraphOutput(input_node),
                block: None,
                state: state.clone(),
                facing_diode: false,
                comparator_far_input: None,
                is_input: false,
                is_output: true,
            });
            graph[input_node].ty = NodeType::SubgraphInput(output_node);

            let edge = graph.remove_edge(edge_idx).unwrap();
            graph.add_edge(
                start_node_idx,
                output_node,
                CompileLink {
                    ty: LinkType::Default,
                    ss: 0,
                },
            );
            graph.add_edge(input_node, end_node_idx, edge);
        }
    }
}
