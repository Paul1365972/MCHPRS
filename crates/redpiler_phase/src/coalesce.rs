use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::analysis::ss_sets::{self, ss_sets, SSSet};
use mchprs_redpiler::behavior::InputStrengths;
use mchprs_redpiler::circuit::{inputs_in, is_stale};
use mchprs_redpiler::graph::{Direction, Graph, NodeIdx, NodeKind, NodeState};
use mchprs_redpiler::passes::coalesce::twins::{merge, Signature};
use mchprs_redpiler::passes::Pass;
use mchprs_redstone::tick_queue::PendingTick;
use mchprs_simulation::ports::Role;
use rustc_hash::FxHashMap;
use std::cmp::Reverse;

#[derive(Debug)]
pub struct Coalesce;

impl Pass for Coalesce {
    fn name(&self) -> &'static str {
        "coalesce"
    }

    fn run(&self, graph: &mut Graph) {
        merge_all(graph, twin_classes(graph));
        let alike = alike_comparators(graph);
        if !alike.is_empty() {
            merge_all(graph, alike);
            merge_all(graph, twin_classes(graph));
        }
    }
}

fn merge_all(graph: &mut Graph, classes: Vec<Vec<NodeIdx>>) {
    for members in classes {
        if let Some(&class) = members.first() {
            merge(graph, class, &members);
        }
    }
}

#[derive(PartialEq, Eq, Hash)]
struct SingleSourceComparator {
    source: NodeIdx,
    outputs_at_min_max: [SignalStrength; 2],
    prioritized: bool,
    state: NodeState,
    pending_tick: Option<PendingTick>,
    port: Option<Role>,
    observed: bool,
}

fn alike_comparators(graph: &Graph) -> Vec<Vec<NodeIdx>> {
    let sets = ss_sets(graph);
    let mut classes: FxHashMap<SingleSourceComparator, Vec<NodeIdx>> = FxHashMap::default();
    for idx in graph.node_indices() {
        if let Some(key) = SingleSourceComparator::of(graph, &sets, idx) {
            classes.entry(key).or_default().push(idx);
        }
    }
    classes
        .into_values()
        .filter(|members| members.len() > 1)
        .collect()
}

impl SingleSourceComparator {
    fn of(graph: &Graph, sets: &ss_sets::SSSets, idx: NodeIdx) -> Option<Self> {
        let node = &graph[idx];
        let NodeKind::Comparator { prioritized, .. } = node.kind else {
            return None;
        };
        if node.is_input() || node.pending_tick.is_none() && is_stale(node, inputs_in(graph, idx)) {
            return None;
        }
        let links: Vec<_> = graph.edges(idx, Direction::Incoming).collect();
        let mut changing = links
            .iter()
            .filter(|edge| {
                !sets
                    .of(edge.source())
                    .decay(edge.weight().weight)
                    .is_single()
            })
            .map(|edge| edge.source());
        let source = changing.next()?;
        if changing.any(|other| other != source) {
            return None;
        }
        let strengths = sets.of(source);
        let ends = [strengths.min(), strengths.max()];
        if strengths != SSSet::singleton(ends[0]).union(SSSet::singleton(ends[1])) {
            return None;
        }
        let outputs_at_min_max = ends.map(|end| {
            let end = SignalStrength::try_from(end).expect("a strength set holds strengths");
            let mut inputs = InputStrengths::default();
            for edge in &links {
                let power = match edge.source() == source {
                    true => end,
                    false => graph[edge.source()].state.power,
                };
                inputs.raise(
                    edge.weight().kind,
                    power.saturating_sub(edge.weight().weight),
                );
            }
            node.kind.tick(node.state, inputs).state.power
        });
        Some(Self {
            source,
            outputs_at_min_max,
            prioritized,
            state: node.state,
            pending_tick: node.pending_tick.map(|queued| queued.tick),
            port: node.port,
            observed: node.observed,
        })
    }
}

#[derive(PartialEq, Eq, Hash)]
struct Likeness {
    signature: Signature<usize>,
    pending_tick: Option<PendingTick>,
    input: Option<NodeIdx>,
}

impl Likeness {
    fn of(graph: &Graph, idx: NodeIdx, class_of: &[usize]) -> Likeness {
        let node = &graph[idx];
        Likeness {
            signature: Signature::of(graph, idx, class_of),
            pending_tick: node.pending_tick.map(|queued| queued.tick),
            input: node.is_input().then_some(idx),
        }
    }
}

struct Class {
    count: usize,
    likeness: Option<Likeness>,
}

fn twin_classes(graph: &Graph) -> Vec<Vec<NodeIdx>> {
    let mut class_of = vec![0; graph.node_bound()];
    let mut classes = vec![Class {
        count: graph.node_count(),
        likeness: None,
    }];
    let mut unsettled: Vec<NodeIdx> = graph.node_indices().collect();
    while !unsettled.is_empty() {
        unsettled.sort_unstable();
        unsettled.dedup();
        let mut parts: FxHashMap<(usize, Likeness), Vec<NodeIdx>> = FxHashMap::default();
        for node in unsettled.drain(..) {
            let class = class_of[node.index()];
            classes[class].count -= 1;
            let likeness = Likeness::of(graph, node, &class_of);
            parts.entry((class, likeness)).or_default().push(node);
        }
        let mut largest_parts_first: Vec<_> = parts.into_iter().collect();
        largest_parts_first.sort_by_key(|(_, part)| Reverse(part.len()));
        for ((class, likeness), part) in largest_parts_first {
            let existing = &mut classes[class];
            if existing.count == 0 || existing.likeness.as_ref() == Some(&likeness) {
                existing.count += part.len();
                existing.likeness = Some(likeness);
                continue;
            }
            for &node in &part {
                class_of[node.index()] = classes.len();
                unsettled.extend(graph.neighbors(node, Direction::Outgoing));
            }
            classes.push(Class {
                count: part.len(),
                likeness: Some(likeness),
            });
        }
    }
    let mut members = vec![Vec::new(); classes.len()];
    for node in graph.node_indices() {
        members[class_of[node.index()]].push(node);
    }
    members
}
