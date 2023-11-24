//! # [`ApplySeparators`]
//!
//! Apply and replace all separator nodes with input and a corresponding output node

use super::Pass;
use crate::redpiler::compile_graph::{
    Annotations, CompileGraph, CompileLink, CompileNode, NodeIdx, NodeType,
};
use crate::redpiler::{BackendVariant, CompilerInput, CompilerOptions};
use crate::world::World;
use itertools::Itertools;
use petgraph::visit::{EdgeRef, NodeIndexable};
use petgraph::Direction;

pub struct ApplySeparators;

impl<W: World> Pass<W> for ApplySeparators {
    fn run_pass(&self, graph: &mut CompileGraph, _: &CompilerOptions, _: &CompilerInput<'_, W>) {
        for i in 0..graph.node_bound() {
            let idx = NodeIdx::new(i);
            if !graph.contains_node(idx) {
                continue;
            }

            let node = &graph[idx];
            if let Some(delay) = node.annotations.separate {
                let state = node.state.clone();

                let input_node = graph.add_node(CompileNode {
                    ty: NodeType::ExternalInput,
                    block: None,
                    state: state.clone(),
                    facing_diode: false,
                    comparator_far_input: None,
                    is_input: true,
                    is_output: false,
                    annotations: Annotations::default(),
                });
                let output_node = graph.add_node(CompileNode {
                    ty: NodeType::ExternalOutput {
                        target_idx: input_node,
                        delay,
                    },
                    block: None,
                    state: state.clone(),
                    facing_diode: false,
                    comparator_far_input: None,
                    is_input: false,
                    is_output: true,
                    annotations: Annotations::default(),
                });

                let incoming_edges = graph
                    .edges_directed(idx, Direction::Incoming)
                    .map(|e| (e.source(), e.weight().ty, e.weight().ss))
                    .collect_vec();
                for (source, ty, ss) in incoming_edges {
                    graph.add_edge(source, output_node, CompileLink { ty, ss });
                }

                let outgoing_edges = graph
                    .edges_directed(idx, Direction::Outgoing)
                    .map(|e| (e.target(), e.weight().ty, e.weight().ss))
                    .collect_vec();
                for (target, ty, ss) in outgoing_edges {
                    graph.add_edge(input_node, target, CompileLink { ty, ss });
                }

                graph.remove_node(idx);
            }
        }
    }

    fn should_run(&self, options: &CompilerOptions) -> bool {
        options.backend_variant == BackendVariant::Threading
    }
}
