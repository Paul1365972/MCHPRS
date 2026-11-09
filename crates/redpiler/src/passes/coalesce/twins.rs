use crate::graph::{Direction, Graph, LinkKind, NodeIdx, NodeKind, NodeState, SignalKind};
use mchprs_simulation::ports::Role;
use smallvec::SmallVec;

#[derive(Debug, PartialEq, Eq, Hash)]
pub struct Signature<Class = NodeIdx> {
    kind: NodeKind,
    state: NodeState,
    port: Option<Role>,
    observed: bool,
    inputs: SmallVec<[(Class, LinkKind, u8); 4]>,
}

impl<Class: Copy + Ord> Signature<Class> {
    pub fn of(graph: &Graph, idx: NodeIdx, class_of: &[Class]) -> Self {
        let node = &graph[idx];
        let mut inputs: SmallVec<_> = graph
            .edges(idx, Direction::Incoming)
            .map(|edge| {
                let link = edge.weight();
                let weight = effective_weight(graph, edge.source(), idx, link.weight);
                (class_of[edge.source().index()], link.kind, weight)
            })
            .collect();
        inputs.sort_unstable();
        inputs.dedup_by_key(|&mut (source, kind, _)| (source, kind));
        Signature {
            kind: node.kind,
            state: node.state,
            port: node.port,
            observed: node.observed,
            inputs,
        }
    }

    pub fn is_powered_by(&self, class: Class) -> bool {
        self.inputs.iter().any(|&(source, ..)| source == class)
    }
}

fn effective_weight(graph: &Graph, source: NodeIdx, target: NodeIdx, weight: u8) -> u8 {
    let source = &graph[source];
    let carries_on_or_off = source.kind.traits().emits == Some(SignalKind::OnOff)
        || source.kind == NodeKind::Constant && weight < source.state.power.get();
    if graph[target].kind.traits().reads == Some(SignalKind::OnOff) && carries_on_or_off {
        0
    } else {
        weight
    }
}

/// Makes `class`, one of `members`, stand for all of them.
pub fn merge(graph: &mut Graph, class: NodeIdx, members: &[NodeIdx]) {
    let mut blocks = SmallVec::new();
    let mut names = Vec::new();
    for &member in members {
        blocks.append(&mut graph[member].blocks);
        names.append(&mut graph[member].names);
        if member != class {
            for (target, link) in graph.remove_edges(member, Direction::Outgoing) {
                graph.add_link(class, target, link);
            }
            graph.remove_node(member);
        }
    }
    graph[class].blocks = blocks;
    graph[class].names = names;
}
