//! # [`UnifyConstants`]
//!
//! Moves the links of every constant onto one constant of strength 15, each lengthened by the
//! strength its constant lacks, so links carrying the same strength come from the same node.
//! Constants never change, so their blocks need no writing back, and every constant that isn't a
//! port goes.

use crate::graph::{Direction, Graph, Link, LinkKind, Node, NodeIdx, NodeKind, NodeState};
use crate::passes::Pass;
use mchprs_blocks::signal_strength::SignalStrength;
use rustc_hash::FxHashMap;
use smallvec::SmallVec;
use tracing::trace;

pub struct UnifyConstants;

impl Pass for UnifyConstants {
    fn run(&self, graph: &mut Graph) {
        let constants: Vec<NodeIdx> = graph
            .node_indices()
            .filter(|&idx| graph[idx].kind == NodeKind::Constant)
            .collect();
        let mut weights: FxHashMap<(NodeIdx, LinkKind), u8> = FxHashMap::default();
        for &constant in &constants {
            let strength = graph[constant].state.power.get();
            for edge in graph.edges(constant, Direction::Outgoing) {
                let link = edge.weight();
                if link.weight < strength {
                    let weight = link.weight + SignalStrength::Fifteen.get() - strength;
                    let shortest = weights.entry((edge.target(), link.kind)).or_insert(weight);
                    *shortest = (*shortest).min(weight);
                }
            }
        }
        let mut num_removed = 0;
        for constant in constants {
            if graph[constant].port.is_some() {
                let mut outgoing = graph.neighbors(constant, Direction::Outgoing).detach();
                while let Some(edge) = outgoing.next_edge(graph) {
                    graph.remove_edge(edge);
                }
            } else {
                graph.remove_node(constant);
                num_removed += 1;
            }
        }
        if !weights.is_empty() {
            let unified = graph.add_node(Node {
                kind: NodeKind::Constant,
                blocks: SmallVec::new(),
                names: Vec::new(),
                state: NodeState::from_power(SignalStrength::Fifteen),
                pending_tick: None,
                port: None,
                observed: false,
            });
            let mut links: Vec<_> = weights.into_iter().collect();
            links.sort_unstable();
            for ((target, kind), weight) in links {
                graph.add_edge(unified, target, Link::new(kind, weight));
            }
        }
        trace!("Removed {} constants", num_removed);
    }

    fn status_message(&self) -> &'static str {
        "Unifying constants"
    }

    fn name(&self) -> &'static str {
        "unify-constants"
    }
}
