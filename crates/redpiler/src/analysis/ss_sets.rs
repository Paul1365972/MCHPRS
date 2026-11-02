//! # [`ss_sets`]
//!
//! This analysis approximates output strengths using the compile graph's node states and inputs.
//! Every node starts with its current output strength, and an input with every strength the
//! caller can set it to. Sweeps in index order grow each set by what
//! the sets of its inputs allow until a sweep changes nothing. Sets only grow, so the sweeps
//! terminate and can retain tighter bounds through cycles. Correlations between inputs are not
//! tracked.
//!
//! A queued repeater tick can emit a pulse after its input disappears, so pending repeaters
//! also admit both 0 and 15.

use crate::graph::{Direction, Drive, Graph, LinkKind, NodeIdx, NodeKind, NodeState};
use mchprs_blocks::blocks::ComparatorMode;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct SSSet(u16);

impl SSSet {
    pub const ZERO: SSSet = SSSet(1);
    pub const BINARY: SSSet = SSSet(1 | 1 << 15);

    const ALL: SSSet = SSSet(u16::MAX);

    fn drivable_strengths(kind: &NodeKind) -> SSSet {
        match kind.drive() {
            Some(Drive::Strength) => SSSet::ALL,
            Some(Drive::OnOff | Drive::Press) | None => SSSet::BINARY,
        }
    }

    pub fn singleton(ss: u8) -> SSSet {
        SSSet(1 << ss)
    }

    fn binary(can_be_off: bool, can_be_on: bool) -> SSSet {
        SSSet(can_be_off as u16 | (can_be_on as u16) << 15)
    }

    pub fn contains(self, ss: u8) -> bool {
        self.0 & (1 << ss) != 0
    }

    pub fn min(self) -> u8 {
        debug_assert!(self.0 != 0);
        self.0.trailing_zeros() as u8
    }

    pub fn max(self) -> u8 {
        debug_assert!(self.0 != 0);
        15 - self.0.leading_zeros() as u8
    }

    pub fn union(self, other: SSSet) -> SSSet {
        SSSet(self.0 | other.0)
    }

    /// The strongest of two inputs is one of the two and at least as strong as both minima
    pub fn combine(self, other: SSSet) -> SSSet {
        let threshold = self.min().max(other.min());
        SSSet((self.0 | other.0) & !((1 << threshold) - 1))
    }

    pub fn decay(self, distance: u8) -> SSSet {
        if distance > 15 {
            return SSSet::ZERO;
        }
        SSSet((self.0 >> distance) | (self.min() < distance) as u16)
    }

    pub fn saturating_sub(self, side: SSSet) -> SSSet {
        (0..16)
            .filter(|&ss| side.contains(ss))
            .map(|ss| self.decay(ss))
            .reduce(SSSet::union)
            .unwrap()
    }

    pub fn saturating_compare(self, side: SSSet) -> SSSet {
        let passing = self.0 & !((1 << side.min()) - 1);
        SSSet(passing | (self.min() < side.max()) as u16)
    }

    pub fn far_override(self, far_input: u8) -> SSSet {
        let far = if self.min() < 15 {
            SSSet::singleton(far_input).0
        } else {
            0
        };
        let full = if self.contains(15) { 1 << 15 } else { 0 };
        SSSet(far | full)
    }

    pub fn to_binary(self) -> SSSet {
        SSSet::binary(self.contains(0), self.max() > 0)
    }

    pub fn to_inverted_binary(self) -> SSSet {
        SSSet::binary(self.max() > 0, self.contains(0))
    }
}

pub struct SSSets(Vec<Option<SSSet>>);

impl SSSets {
    pub fn of(&self, node_idx: NodeIdx) -> SSSet {
        self.0[node_idx.index()].expect("every node of the analyzed graph has a set")
    }
}

pub fn ss_sets(graph: &Graph) -> SSSets {
    let nodes: Vec<NodeIdx> = graph.node_indices().collect();
    let mut sets = vec![None; graph.node_bound()];
    let mut dirty = vec![true; graph.node_bound()];
    for &node_idx in &nodes {
        let node = &graph[node_idx];
        let initial = if node.is_input() {
            SSSet::drivable_strengths(&node.kind)
        } else if node.pending_tick.is_some() && matches!(node.kind, NodeKind::Repeater { .. }) {
            SSSet::BINARY
        } else {
            SSSet::singleton(node.state.power.get())
        };
        sets[node_idx.index()] = Some(initial);
    }

    let mut changed = true;
    while changed {
        changed = false;
        for &node_idx in &nodes {
            if !dirty[node_idx.index()] {
                continue;
            }
            dirty[node_idx.index()] = false;

            let (default_input, side_input) = collect_inputs(graph, &sets, node_idx);
            let node = &graph[node_idx];
            let old = sets[node_idx.index()].unwrap();
            let new = old.union(evaluate(&node.kind, &node.state, default_input, side_input));
            if new == old {
                continue;
            }
            sets[node_idx.index()] = Some(new);
            changed = true;

            for neighbor in graph.neighbors(node_idx, Direction::Outgoing) {
                dirty[neighbor.index()] = true;
            }
        }
    }

    SSSets(sets)
}

fn collect_inputs(graph: &Graph, sets: &[Option<SSSet>], node_idx: NodeIdx) -> (SSSet, SSSet) {
    let mut default_input = SSSet::ZERO;
    let mut side_input = SSSet::ZERO;
    for edge in graph.edges(node_idx, Direction::Incoming) {
        let link = edge.weight();
        let source = sets[edge.source().index()].unwrap().decay(link.weight);
        match link.kind {
            LinkKind::Default => default_input = default_input.combine(source),
            LinkKind::Side => side_input = side_input.combine(source),
        }
    }
    (default_input, side_input)
}

fn evaluate(kind: &NodeKind, state: &NodeState, default_input: SSSet, side_input: SSSet) -> SSSet {
    match kind {
        NodeKind::Repeater { .. } => {
            if side_input.contains(0) {
                default_input.to_binary()
            } else {
                SSSet::singleton(state.power.get())
            }
        }
        NodeKind::Torch => default_input.to_inverted_binary(),
        NodeKind::Lamp | NodeKind::Trapdoor | NodeKind::NoteBlock { .. } => {
            default_input.to_binary()
        }
        NodeKind::Comparator {
            mode, far_input, ..
        } => {
            let input = match far_input {
                Some(far_input) => default_input.far_override(far_input.get()),
                None => default_input,
            };
            match mode {
                ComparatorMode::Compare => input.saturating_compare(side_input),
                ComparatorMode::Subtract => input.saturating_sub(side_input),
            }
        }
        NodeKind::Wire => default_input,
        NodeKind::Constant => SSSet::singleton(state.power.get()),
        NodeKind::Button | NodeKind::Lever | NodeKind::PressurePlate => SSSet::BINARY,
    }
}
