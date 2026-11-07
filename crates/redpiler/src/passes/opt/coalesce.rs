//! # [`Coalesce`]
//!
//! Merges nodes with the same type, initial state and input links, except inputs, nodes with a
//! pending tick and nodes whose inputs could change between their ticks.
//!
//! The merged node updates the targets of one merged node after another. With
//! [`UpdateOrder::Kept`], nodes only merge in runs whose ticks always follow each other: every
//! source updates them in the same order, and no target in between could be scheduled into the same
//! tick and priority, unless its tick commutes with theirs. Nodes that feed nothing merge without
//! that, since their ticks only change themselves.
//!
//! Merges make links parallel and remove the nodes between parallel links. So once no nodes merge,
//! parallel links merge as in [`DedupLinks`](crate::passes::DedupLinks), and the nodes their
//! sources feed are checked again, until nothing merges.

use super::dedup_links::dedup_links;
use super::tick_slots::TickSlots;
use crate::graph::{Direction, Graph, Link, LinkKind, NodeIdx, NodeKind, NodeState};
use crate::passes::{Pass, UpdateOrder};
use crate::ports::Role;
use rustc_hash::{FxHashMap, FxHashSet};
use smallvec::SmallVec;
use tracing::trace;

pub struct Coalesce(pub UpdateOrder);

impl Pass for Coalesce {
    fn run(&self, graph: &mut Graph) {
        let node_count = graph.node_count();
        let mut candidates: Vec<NodeIdx> = graph.node_indices().collect();
        loop {
            let merged_into = self.merge_identical(graph, candidates);
            let changed_sources = if merged_into.is_empty() {
                dedup_links(graph, self.0)
            } else {
                merged_into
            };
            if changed_sources.is_empty() {
                break;
            }
            candidates = targets(graph, changed_sources);
        }
        trace!("Coalesced {} nodes", node_count - graph.node_count());
    }

    fn status_message(&self) -> &'static str {
        "Combining duplicate logic"
    }

    fn name(&self) -> &'static str {
        match self.0 {
            UpdateOrder::Kept => "coalesce",
            UpdateOrder::Free => "coalesce-reorder",
        }
    }
}

impl Coalesce {
    /// Merges identical candidates and returns the nodes they merged into
    fn merge_identical(&self, graph: &mut Graph, candidates: Vec<NodeIdx>) -> Vec<NodeIdx> {
        let mut groups: FxHashMap<Signature, Vec<NodeIdx>> = FxHashMap::default();
        for node in candidates {
            if !graph.contains_node(node)
                || graph[node].is_input()
                || graph[node].pending_tick.is_some()
            {
                continue;
            }
            let signature = signature(graph, node, self.0);
            if inputs_are_stable(graph, &signature, node) {
                groups.entry(signature).or_default().push(node);
            }
        }
        let runs: Vec<Vec<NodeIdx>> = groups
            .into_values()
            .filter(|members| members.len() > 1)
            .flat_map(|members| match self.0 {
                UpdateOrder::Kept => ordered_runs(graph, members),
                UpdateOrder::Free => vec![members],
            })
            .collect();
        let mut merged_into = Vec::with_capacity(runs.len());
        for run in runs {
            let into = run[0];
            for &node in &run[1..] {
                for (target, link) in take_outgoing_links(graph, node) {
                    graph.add_edge(into, target, link);
                }
                let mut removed = graph.remove_node(node).unwrap();
                graph[into].blocks.append(&mut removed.blocks);
                graph[into].names.append(&mut removed.names);
            }
            merged_into.push(into);
        }
        merged_into
    }
}

/// The nodes these sources feed: every node whose inputs changed with them, and every node that
/// could now have the same inputs as one of those
fn targets(graph: &Graph, mut sources: Vec<NodeIdx>) -> Vec<NodeIdx> {
    sources.sort_unstable();
    sources.dedup();
    let mut queued = FxHashSet::default();
    sources
        .into_iter()
        .flat_map(|source| graph.neighbors(source, Direction::Outgoing))
        .filter(|&target| queued.insert(target))
        .collect()
}

#[derive(Debug, PartialEq, Eq, Hash)]
struct Signature {
    kind: NodeKind,
    state: NodeState,
    port: Option<Role>,
    observed: bool,
    inputs: SmallVec<[(NodeIdx, LinkKind, u8); 4]>,
}

/// Counts every input link. Reordering allows counting only the strongest link from each source
/// per link kind, since the others never decide the input.
fn signature(graph: &Graph, idx: NodeIdx, order: UpdateOrder) -> Signature {
    let node = &graph[idx];
    let mut inputs: SmallVec<_> = graph
        .edges(idx, Direction::Incoming)
        .map(|edge| {
            let link = edge.weight();
            let source = &graph[edge.source()];
            let carries_on_or_off = source.kind.is_binary_source()
                || source.kind == NodeKind::Constant && link.weight < source.state.power.get();
            let weight = if node.kind.is_binary_reader() && carries_on_or_off {
                0
            } else {
                link.weight
            };
            (edge.source(), link.kind, weight)
        })
        .collect();
    inputs.sort_unstable();
    if order == UpdateOrder::Free {
        inputs.dedup_by_key(|&mut (source, kind, _)| (source, kind));
    }
    Signature {
        kind: node.kind.clone(),
        state: node.state,
        port: node.port,
        observed: node.observed,
        inputs,
    }
}

/// An input updated by another input can tick between two identical nodes and change in between
fn inputs_are_stable(graph: &Graph, signature: &Signature, node: NodeIdx) -> bool {
    let is_input = |idx| signature.inputs.iter().any(|&(source, ..)| source == idx);
    !is_input(node)
        && signature
            .inputs
            .iter()
            .all(|&(source, ..)| !graph.neighbors(source, Direction::Incoming).any(is_input))
}

struct Group {
    members: FxHashSet<NodeIdx>,
    slots: TickSlots,
    targets: FxHashSet<NodeIdx>,
    caused_tick_slots: TickSlots,
}

impl Group {
    fn new(graph: &Graph, members: &[NodeIdx]) -> Group {
        Group {
            members: members.iter().copied().collect(),
            slots: TickSlots::of(&graph[members[0]].kind),
            targets: members
                .iter()
                .flat_map(|&member| graph.neighbors(member, Direction::Outgoing))
                .collect(),
            caused_tick_slots: members.iter().fold(TickSlots::default(), |slots, &member| {
                slots.union(caused_tick_slots(graph, member))
            }),
        }
    }

    /// Whether a tick of this target, updated between two members, could run between their ticks
    /// and change what follows
    fn separates_members(&self, graph: &Graph, target: NodeIdx) -> bool {
        !self.members.contains(&target)
            && graph[target].kind.is_link_source()
            && TickSlots::of(&graph[target].kind).intersects(self.slots)
            && (self.targets.contains(&target)
                || graph
                    .neighbors(target, Direction::Outgoing)
                    .any(|next| self.members.contains(&next) || self.targets.contains(&next))
                || caused_tick_slots(graph, target).intersects(self.caused_tick_slots))
    }
}

struct UpdateSpan {
    first: usize,
    last: usize,
}

struct SourceUpdates {
    targets: Vec<NodeIdx>,
    member_spans: FxHashMap<NodeIdx, UpdateSpan>,
}

/// Where the ticks a tick of this node causes are scheduled, except those of nodes that feed
/// nothing
fn caused_tick_slots(graph: &Graph, node: NodeIdx) -> TickSlots {
    graph
        .neighbors(node, Direction::Outgoing)
        .filter(|&target| graph[target].kind.is_link_source())
        .fold(
            TickSlots::of_own_tick(&graph[node].kind),
            |slots, target| slots.union(TickSlots::of(&graph[target].kind)),
        )
}

/// Splits identical nodes into runs whose ticks always follow each other, each in the order they
/// tick in
fn ordered_runs(graph: &Graph, mut members: Vec<NodeIdx>) -> Vec<Vec<NodeIdx>> {
    let kind = &graph[members[0]].kind;
    let mut sources: Vec<NodeIdx> = graph
        .neighbors(members[0], Direction::Incoming)
        .filter(|&source| graph[source].kind != NodeKind::Constant)
        .collect();
    sources.sort_unstable();
    sources.dedup();
    if !kind.is_link_source() || sources.is_empty() {
        return vec![members];
    }
    let group = Group::new(graph, &members);
    let updates: Vec<SourceUpdates> = sources
        .into_iter()
        .map(|source| {
            let targets: Vec<NodeIdx> = graph.neighbors(source, Direction::Outgoing).collect();
            let mut member_spans: FxHashMap<NodeIdx, UpdateSpan> = FxHashMap::default();
            for (position, &target) in targets.iter().enumerate() {
                if group.members.contains(&target) {
                    member_spans
                        .entry(target)
                        .or_insert(UpdateSpan {
                            first: position,
                            last: position,
                        })
                        .last = position;
                }
            }
            SourceUpdates {
                targets,
                member_spans,
            }
        })
        .collect();
    members.sort_by_key(|member| updates[0].member_spans[member].first);
    let mut split_before = vec![false; members.len()];
    for SourceUpdates {
        targets,
        member_spans,
    } in &updates
    {
        for i in 1..members.len() {
            let previous = &member_spans[&members[i - 1]];
            let next = &member_spans[&members[i]];
            if previous.last >= next.first {
                return Vec::new();
            }
            split_before[i] |= targets[previous.last + 1..next.first]
                .iter()
                .any(|&target| group.separates_members(graph, target));
        }
    }
    let mut runs = vec![vec![members[0]]];
    for (&member, &split) in members.iter().zip(&split_before).skip(1) {
        if split {
            runs.push(Vec::new());
        }
        runs.last_mut().unwrap().push(member);
    }
    runs.retain(|run| run.len() > 1);
    runs
}

fn take_outgoing_links(graph: &mut Graph, node: NodeIdx) -> Vec<(NodeIdx, Link)> {
    let mut links = Vec::new();
    let mut outgoing = graph.neighbors(node, Direction::Outgoing).detach();
    while let Some((edge_idx, target)) = outgoing.next(graph) {
        links.push((target, graph.remove_edge(edge_idx).unwrap()));
    }
    links
}
