use super::Queues;
use crate::graph::{Direction, NodeIdx, NodeKind};
use itertools::{iproduct, Itertools};
use rustc_hash::{FxHashMap, FxHashSet};

type Pair = (NodeIdx, NodeIdx);

#[derive(Default)]
pub struct Independence {
    known: FxHashMap<Pair, bool>,
}

impl Independence {
    const MOST_PAIRS_FOLLOWED: usize = 512;

    pub fn holds(&mut self, queues: &Queues, a: NodeIdx, b: NodeIdx) -> bool {
        let start = pair(a, b);
        if let Some(&known) = self.known.get(&start) {
            return known;
        }
        let mut found = FxHashSet::from_iter([start]);
        let mut open = vec![start];
        while let Some((a, b)) = open.pop() {
            let known = self.known.get(&(a, b)).copied();
            let is_dependent = match known {
                Some(independent) => !independent,
                None => found.len() > Self::MOST_PAIRS_FOLLOWED || interact(queues, a, b),
            };
            if is_dependent {
                self.known.insert(start, false);
                return false;
            }
            if known.is_none() {
                let scheduled = iproduct!(scheduled_by(queues, a), scheduled_by(queues, b));
                let together = scheduled.filter(|&(a, b)| queues.can_share(a, b));
                let next = together.map(|(a, b)| pair(a, b));
                open.extend(next.filter(|&next| found.insert(next)));
            }
        }
        let independent = found.into_iter().map(|pair| (pair, true));
        self.known.extend(independent);
        true
    }
}

fn pair(a: NodeIdx, b: NodeIdx) -> Pair {
    (a.min(b), a.max(b))
}

fn interact(queues: &Queues, a: NodeIdx, b: NodeIdx) -> bool {
    let targets = |node: NodeIdx| queues.graph.neighbors(node, Direction::Outgoing);
    targets(b).contains(&a) || targets(a).any(|target| target == b || targets(b).contains(&target))
}

/// Of the kinds a link can end at, only a repeater schedules itself in its tick.
fn scheduled_by(queues: &Queues, node: NodeIdx) -> Vec<NodeIdx> {
    let pulses = matches!(queues.graph[node].kind, NodeKind::Repeater { .. });
    let targets = queues.graph.neighbors(node, Direction::Outgoing);
    let queued = targets.filter(|&target| queues.can_hold(target));
    queued.chain(pulses.then_some(node)).unique().collect()
}
