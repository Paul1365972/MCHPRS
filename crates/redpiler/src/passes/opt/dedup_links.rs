//! # [`DedupLinks`]
//!
//! This pass removes duplicate edges from the graph, or parallel edges that have higher weight.
//!
//! For example, if two nodes are connected with two links of weights 13 and 15, the link with
//! weight 15 is removed.
//!
//! With [`UpdateOrder::Kept`], parallel links only merge where that can't change an update. A
//! source updates its links into one target one after another, and in between the target sees the
//! new strength on one link and the old one on the other. A reader of on or off then sees either
//! its old or its final input, which changes nothing in a reader that agrees with its inputs, and a
//! wire shows only its final strength. The link that stays updates the target earlier than the last
//! one did, so no target between them in the source's order may be able to tick in the same tick
//! and priority.

use super::tick_slots::TickSlots;
use crate::graph::{Direction, EdgeIdx, Graph, LinkKind, NodeIdx, NodeKind};
use crate::passes::{Pass, UpdateOrder};
use smallvec::SmallVec;

pub struct DedupLinks(pub UpdateOrder);

impl Pass for DedupLinks {
    fn run(&self, graph: &mut Graph) {
        dedup_links(graph, self.0);
    }

    fn status_message(&self) -> &'static str {
        "Deduplicating links"
    }

    fn name(&self) -> &'static str {
        match self.0 {
            UpdateOrder::Kept => "dedup-links",
            UpdateOrder::Free => "dedup-links-reorder",
        }
    }
}

/// Merges the parallel links `order` allows merging and returns their sources
pub(super) fn dedup_links(graph: &mut Graph, order: UpdateOrder) -> Vec<NodeIdx> {
    let mut sources = Vec::new();
    for target in (0..graph.node_bound()).map(NodeIdx::new) {
        if graph.contains_node(target) {
            sources.extend(dedup_links_into(graph, target, order));
        }
    }
    sources
}

fn dedup_links_into(
    graph: &mut Graph,
    target: NodeIdx,
    order: UpdateOrder,
) -> SmallVec<[NodeIdx; 4]> {
    let mut sources = SmallVec::new();
    if order == UpdateOrder::Kept && !takes_merged_links(graph, target) {
        return sources;
    }
    let mut links: SmallVec<[(NodeIdx, LinkKind, u8, EdgeIdx); 8]> = graph
        .edges(target, Direction::Incoming)
        .map(|edge| {
            (
                edge.source(),
                edge.weight().kind,
                edge.weight().weight,
                edge.id(),
            )
        })
        .collect();
    links.sort_unstable_by_key(|&(source, kind, weight, _)| (source, kind, weight));
    for parallel in links.chunk_by(|a, b| (a.0, a.1) == (b.0, b.1)) {
        if parallel.len() < 2 {
            continue;
        }
        let edges: SmallVec<[EdgeIdx; 4]> = parallel.iter().map(|link| link.3).collect();
        if order == UpdateOrder::Kept && !updates_in_turn(graph, parallel[0].0, target, &edges) {
            continue;
        }
        for &edge in &edges[1..] {
            graph.remove_edge(edge);
        }
        sources.push(parallel[0].0);
    }
    sources
}

fn takes_merged_links(graph: &Graph, idx: NodeIdx) -> bool {
    let kind = &graph[idx].kind;
    *kind == NodeKind::Wire
        || (kind.is_binary_reader() || !kind.is_link_source()) && agrees_with_inputs(graph, idx)
}

fn agrees_with_inputs(graph: &Graph, idx: NodeIdx) -> bool {
    let node = &graph[idx];
    if node.pending_tick.is_some() {
        return false;
    }
    let powered = |kind| {
        graph.edges(idx, Direction::Incoming).any(|edge| {
            edge.weight().kind == kind
                && graph[edge.source()].state.power.get() > edge.weight().weight
        })
    };
    let on = node.state.is_powered();
    match node.kind {
        NodeKind::Repeater { .. } => {
            let locked = powered(LinkKind::Side);
            node.state.repeater_locked == locked && (locked || on == powered(LinkKind::Default))
        }
        NodeKind::Torch => on != powered(LinkKind::Default),
        NodeKind::Lamp | NodeKind::Trapdoor | NodeKind::NoteBlock { .. } => {
            on == powered(LinkKind::Default)
        }
        _ => false,
    }
}

/// Whether nothing between the source's links to the target could tick between their updates
fn updates_in_turn(graph: &Graph, source: NodeIdx, target: NodeIdx, links: &[EdgeIdx]) -> bool {
    if !graph[target].kind.is_link_source() {
        return true;
    }
    let slots = TickSlots::of(&graph[target].kind);
    let outgoing: Vec<(EdgeIdx, NodeIdx)> = graph
        .edges(source, Direction::Outgoing)
        .map(|edge| (edge.id(), edge.target()))
        .collect();
    let positions = outgoing
        .iter()
        .enumerate()
        .filter(|(_, (edge, _))| links.contains(edge))
        .map(|(position, _)| position);
    let (first, last) = positions.fold((usize::MAX, 0), |(first, last), position| {
        (first.min(position), last.max(position))
    });
    outgoing[first..=last].iter().all(|&(edge, other)| {
        if other == target {
            links.contains(&edge)
        } else {
            !graph[other].kind.is_link_source()
                || !TickSlots::of(&graph[other].kind).intersects(slots)
        }
    })
}
