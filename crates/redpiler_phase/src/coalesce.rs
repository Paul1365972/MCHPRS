use mchprs_redpiler::graph::{Direction, Graph, NodeIdx};
use mchprs_redpiler::passes::coalesce::twins::{merge, Signature};
use mchprs_redpiler::passes::Pass;
use mchprs_redstone::tick_queue::PendingTick;
use rustc_hash::FxHashMap;
use std::cmp::Reverse;

#[derive(Debug)]
pub struct Coalesce;

impl Pass for Coalesce {
    fn name(&self) -> &'static str {
        "coalesce"
    }

    fn run(&self, graph: &mut Graph) {
        for members in twin_classes(graph) {
            if let Some(&class) = members.first() {
                merge(graph, class, &members);
            }
        }
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
