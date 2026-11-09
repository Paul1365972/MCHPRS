mod independence;
pub mod twins;

use super::Pass;
use crate::graph::{Direction, Graph, LinkKind, NodeIdx, NodeKind};
use independence::Independence;
use itertools::Itertools;
use mchprs_redstone::tick_queue::PendingTick;
use rustc_hash::FxHashMap;
use smallvec::SmallVec;
use std::collections::BTreeMap;
use std::slice;
use tracing::trace;
use twins::{merge, Signature};

#[derive(Debug)]
pub struct Coalesce;

impl Pass for Coalesce {
    fn name(&self) -> &'static str {
        "coalesce"
    }

    fn run(&self, graph: &mut Graph) {
        let node_count = graph.node_count();
        let mut planner = Planner::new(graph);
        let mut candidates: Vec<NodeIdx> = graph.node_indices().collect();
        while !candidates.is_empty() {
            let grown = planner.merge_identical(candidates);
            candidates = planner.targets(grown);
        }
        let (merged, link_orders) = (planner.merged, planner.link_orders);
        for (&class, members) in &merged {
            merge(graph, class, members);
        }
        for (class, order) in link_orders {
            if merged.contains_key(&class) {
                let places: FxHashMap<MergedLink, usize> = order.into_iter().zip(0..).collect();
                graph.sort_edges_by_key(class, Direction::Outgoing, |_, link| {
                    places[&(link.target(), link.weight().kind)]
                });
            }
        }
        trace!("Coalesced {} nodes", node_count - graph.node_count());
    }
}

type MergedLink = (NodeIdx, LinkKind);

struct Entry {
    target: NodeIdx,
    kind: LinkKind,
    member: u32,
    list_place: u32,
}

struct Queues<'a> {
    graph: &'a Graph,
    ticks: Vec<SmallVec<[PendingTick; 2]>>,
}

impl<'a> Queues<'a> {
    fn new(graph: &'a Graph) -> Queues<'a> {
        let mut of_kinds: FxHashMap<&NodeKind, Vec<PendingTick>> = FxHashMap::default();
        let mut ticks = vec![SmallVec::new(); graph.node_bound()];
        for node in graph.node_indices() {
            let kind = &graph[node].kind;
            let of_kind = of_kinds.entry(kind);
            let of_kind = of_kind.or_insert_with(|| kind.schedulable_ticks());
            ticks[node.index()] = SmallVec::from_slice(of_kind);
        }
        Queues { graph, ticks }
    }

    fn can_hold(&self, node: NodeIdx) -> bool {
        !self.ticks[node.index()].is_empty()
    }

    fn can_share(&self, a: NodeIdx, b: NodeIdx) -> bool {
        let (of_a, of_b) = (&self.ticks[a.index()], &self.ticks[b.index()]);
        of_a.iter().any(|tick| of_b.contains(tick))
    }
}

struct Planner<'a> {
    graph: &'a Graph,
    queues: Queues<'a>,
    independence: Independence,
    class_of: Vec<NodeIdx>,
    place_of: Vec<u32>,
    merged: BTreeMap<NodeIdx, Vec<NodeIdx>>,
    link_orders: FxHashMap<NodeIdx, Vec<MergedLink>>,
}

impl<'a> Planner<'a> {
    const MOST_ENTRIES_COMPARED: usize = 1024;

    fn new(graph: &'a Graph) -> Planner<'a> {
        Planner {
            graph,
            queues: Queues::new(graph),
            independence: Independence::default(),
            class_of: (0..graph.node_bound()).map(NodeIdx::new).collect(),
            place_of: vec![0; graph.node_bound()],
            merged: BTreeMap::new(),
            link_orders: FxHashMap::default(),
        }
    }

    fn members(&self, class: NodeIdx) -> &[NodeIdx] {
        match self.merged.get(&class) {
            Some(members) => members,
            None => slice::from_ref(&self.class_of[class.index()]),
        }
    }

    fn set(&mut self, class: NodeIdx, members: Vec<NodeIdx>) {
        for (place, &member) in (0..).zip(&members) {
            self.class_of[member.index()] = class;
            self.place_of[member.index()] = place;
        }
        if members.len() > 1 {
            self.merged.insert(class, members);
        }
    }

    fn take(&mut self, class: NodeIdx) -> Vec<NodeIdx> {
        self.merged.remove(&class).unwrap_or_else(|| vec![class])
    }

    fn merge_identical(&mut self, candidates: Vec<NodeIdx>) -> Vec<NodeIdx> {
        let mut groups: FxHashMap<Signature, Vec<NodeIdx>> = FxHashMap::default();
        for class in candidates {
            let node = &self.graph[class];
            if node.is_input() || node.pending_tick.is_some() {
                continue;
            }
            let signature = Signature::of(self.graph, class, &self.class_of);
            if !signature.is_powered_by(class) {
                groups.entry(signature).or_default().push(class);
            }
        }
        let groups = groups.into_values().filter(|group| group.len() > 1);
        let groups = groups.sorted_unstable().collect_vec();
        let grown = groups.iter().filter(|group| self.try_merge(group));
        grown.map(|group| group[0]).collect()
    }

    fn try_merge(&mut self, group: &[NodeIdx]) -> bool {
        let sources = self.graph.neighbors(group[0], Direction::Incoming);
        let sources = sources.map(|source| self.class_of[source.index()]);
        let mut moved = sources.chain([group[0]]).sorted_unstable().collect_vec();
        moved.dedup();
        let parts = group.iter().map(|&class| self.take(class)).collect_vec();
        let mut members = parts.concat();
        members.sort_by_cached_key(|&member| (self.feeder_places(member), member));
        self.set(group[0], members);
        moved.retain(|&class| self.changes_power(class));
        let orders = moved.iter().map(|&class| self.link_order(class));
        let Some(orders) = orders.collect::<Option<Vec<_>>>() else {
            self.take(group[0]);
            for (&class, part) in group.iter().zip(parts) {
                self.set(class, part);
            }
            return false;
        };
        self.link_orders.extend(moved.into_iter().zip(orders));
        true
    }

    fn changes_power(&self, class: NodeIdx) -> bool {
        let traits = self.graph[class].kind.traits();
        self.merged.contains_key(&class) && traits.ticks && traits.emits.is_some()
    }

    /// Members sorted by this are scheduled in the order in which their sources tick.
    fn feeder_places(&self, node: NodeIdx) -> SmallVec<[(NodeIdx, u32); 4]> {
        let feeders = self.graph.neighbors(node, Direction::Incoming);
        let place_of =
            |feeder: NodeIdx| (self.class_of[feeder.index()], self.place_of[feeder.index()]);
        feeders.map(place_of).sorted_unstable().collect()
    }

    fn link_order(&mut self, class: NodeIdx) -> Option<Vec<MergedLink>> {
        let mut entries = self.entries(class)?;
        let (class_of, place_of) = (&self.class_of, &self.place_of);
        let (queues, independence) = (&self.queues, &mut self.independence);
        let link_of = |entry: &Entry| (class_of[entry.target.index()], entry.kind);
        let mut keys: FxHashMap<MergedLink, (u32, u32, u32)> = FxHashMap::default();
        for entry in &entries {
            let key = (entry.member, entry.member, entry.list_place);
            let (first, last, list_place) = keys.entry(link_of(entry)).or_insert(key);
            *first = entry.member.min(*first);
            *last = entry.member.max(*last);
            *list_place = entry.list_place.min(*list_place);
        }
        entries.sort_unstable_by_key(|entry| {
            let link = link_of(entry);
            (keys[&link], link, place_of[entry.target.index()])
        });

        let queued = entries.iter().filter(|entry| queues.can_hold(entry.target));
        let queued = queued.collect_vec();
        let mut may_follow = |earlier: &Entry, later: &Entry| {
            let (a, b) = (earlier.target, later.target);
            earlier.member <= later.member
                || a != b && (!queues.can_share(a, b) || independence.holds(queues, a, b))
        };
        let in_order = queued.is_sorted_by_key(|entry| entry.member)
            || queued.len() <= Self::MOST_ENTRIES_COMPARED
                && queued
                    .iter()
                    .tuple_combinations()
                    .all(|(a, b)| may_follow(a, b));
        in_order.then(|| entries.iter().map(link_of).dedup().collect())
    }

    fn entries(&self, class: NodeIdx) -> Option<Vec<Entry>> {
        let mut entries: FxHashMap<MergedLink, Entry> = FxHashMap::default();
        for (member, &node) in (0..).zip(self.members(class)) {
            let links = self.graph.edges(node, Direction::Outgoing);
            for (list_place, link) in (0..).zip(links) {
                let (target, kind) = (link.target(), link.weight().kind);
                let entry = Entry {
                    target,
                    kind,
                    member,
                    list_place,
                };
                let is_shared = entries.insert((target, kind), entry).is_some();
                if is_shared && self.queues.can_hold(target) {
                    return None;
                }
            }
        }
        Some(entries.into_values().collect())
    }

    fn targets(&self, classes: Vec<NodeIdx>) -> Vec<NodeIdx> {
        let members = classes.iter().flat_map(|&class| self.members(class));
        let targets = members.flat_map(|&member| self.graph.neighbors(member, Direction::Outgoing));
        let targets = targets.map(|target| self.class_of[target.index()]);
        targets.sorted_unstable().dedup().collect()
    }
}
