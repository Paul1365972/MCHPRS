use super::Pass;
use crate::compile_graph::{CompileGraph, CompileNode, NodeIdx, NodeState, NodeType};
use crate::{CompilerInput, CompilerOptions};
use mchprs_world::World;
use petgraph::visit::NodeIndexable;
use petgraph::Direction;
use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::hash_map::Entry;

pub struct ConstantCoalesce;

impl<W: World> Pass<W> for ConstantCoalesce {
    fn run_pass(&self, graph: &mut CompileGraph, _: &CompilerOptions, _: &CompilerInput<'_, W>) {
        let mut constant_nodes: FxHashMap<u8, NodeIdx> = FxHashMap::default();
        let mut constant_nodes_set = FxHashSet::default();

        for i in 0..graph.node_bound() {
            let idx = NodeIdx::new(i);
            if !graph.contains_node(idx) || constant_nodes_set.contains(&idx) {
                continue;
            }
            let node = &graph[idx];
            if node.ty != NodeType::Constant || !node.is_removable() {
                continue;
            }
            let ss = node.state.output_strength;

            let mut targets = graph.neighbors_directed(idx, Direction::Outgoing).detach();
            while let Some((edge, dest)) = targets.next(graph) {
                let weight = graph.remove_edge(edge).unwrap();

                let constant_idx = match constant_nodes.entry(ss) {
                    Entry::Occupied(entry) => *entry.get(),
                    Entry::Vacant(entry) => {
                        let constant_idx = graph.add_node(CompileNode {
                            ty: NodeType::Constant,
                            block: None,
                            state: NodeState::ss(ss),
                            is_input: false,
                            is_output: false,
                            annotations: Default::default(),
                        });
                        constant_nodes_set.insert(constant_idx);
                        entry.insert(constant_idx);
                        constant_idx
                    }
                };
                graph.add_edge(constant_idx, dest, weight);
            }
            graph.remove_node(idx);
        }
    }

    fn status_message(&self) -> &'static str {
        "Coalescing constants"
    }
}
