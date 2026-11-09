use super::Pass;
use crate::graph::{Direction, Graph, Link, LinkKind, Node, NodeIdx, NodeKind, NodeState};
use mchprs_blocks::signal_strength::SignalStrength;
use rustc_hash::FxHashMap;
use smallvec::SmallVec;
use tracing::trace;

#[derive(Debug)]
pub struct UnifyConstants;

impl Pass for UnifyConstants {
    fn name(&self) -> &'static str {
        "unify-constants"
    }

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
        let mut removed = 0;
        for constant in constants {
            if graph[constant].port.is_some() || graph[constant].observed {
                graph.remove_edges(constant, Direction::Outgoing);
            } else {
                graph.remove_node(constant);
                removed += 1;
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
                graph.add_link(unified, target, Link::new(kind, weight));
            }
        }
        trace!("Removed {removed} constants");
    }
}
