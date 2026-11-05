//! # [`UnifyConstants`]
//!
//! Moves the outgoing links of every constant onto one new strength-15 constant, with weights
//! that deliver the same strength. The old constants keep their state and blocks.

use crate::graph::{Direction, Graph, Link, Node, NodeIdx, NodeKind, NodeState};
use crate::passes::Pass;
use mchprs_blocks::signal_strength::SignalStrength;

pub struct UnifyConstants;

impl Pass for UnifyConstants {
    fn run(&self, graph: &mut Graph) {
        let mut links = Vec::new();
        for i in 0..graph.node_bound() {
            let idx = NodeIdx::new(i);
            if !graph.contains_node(idx) || graph[idx].kind != NodeKind::Constant {
                continue;
            }
            let power = graph[idx].state.power;
            let mut outgoing = graph.neighbors(idx, Direction::Outgoing).detach();
            while let Some((edge_idx, target)) = outgoing.next(graph) {
                let link = graph.remove_edge(edge_idx).unwrap();
                let strength = power.saturating_sub(link.weight);
                if !strength.is_zero() {
                    let weight = SignalStrength::Fifteen.get() - strength.get();
                    links.push((target, Link::new(link.kind, weight)));
                }
            }
        }
        if links.is_empty() {
            return;
        }

        let unified = graph.add_node(Node {
            kind: NodeKind::Constant,
            blocks: Default::default(),
            name: None,
            state: NodeState::from_power(SignalStrength::Fifteen),
            pending_tick: None,
            port: None,
            observed: false,
        });
        for (target, link) in links {
            graph.add_edge(unified, target, link);
        }
    }

    fn status_message(&self) -> &'static str {
        "Unifying constants"
    }

    fn name(&self) -> &'static str {
        "unify-constants"
    }
}
