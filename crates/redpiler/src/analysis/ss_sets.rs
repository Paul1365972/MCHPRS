//! The sets only grow, so the sweeps terminate. Correlations between inputs are not tracked.
//!
//! A node without a pending tick whose input links each carry one strength is never updated: it
//! keeps its output, even where that disagrees with its inputs.

use crate::behavior::{AcceptedInput, InputStrengths, Outcome, Pending};
use crate::graph::{Direction, Graph, LinkKind, Node, NodeIdx, NodeKind};
use itertools::iproduct;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redstone::input::Input;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct SSSet(u16);

impl SSSet {
    pub const ZERO: SSSet = SSSet(1);

    const EMPTY: SSSet = SSSet(0);

    pub fn singleton(ss: u8) -> SSSet {
        SSSet(1 << ss)
    }

    pub fn contains(self, ss: u8) -> bool {
        self.0 & (1 << ss) != 0
    }

    pub fn is_single(self) -> bool {
        self.0.is_power_of_two()
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

    /// The strengths the stronger of two inputs from these sets can have
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

    fn strengths(self) -> impl Iterator<Item = SignalStrength> {
        SignalStrength::all().filter(move |power| self.contains(power.get()))
    }

    fn with(self, power: SignalStrength) -> SSSet {
        self.union(SSSet::singleton(power.get()))
    }
}

pub struct SSSets(Vec<Option<SSSet>>);

impl SSSets {
    pub fn of(&self, node_idx: NodeIdx) -> SSSet {
        self.0[node_idx.index()].expect("every node of the analyzed graph has a set")
    }

    pub fn inputs(&self, graph: &Graph, node_idx: NodeIdx) -> InputSets {
        collect_inputs(graph, &self.0, node_idx)
    }
}

pub struct InputSets {
    pub default: SSSet,
    pub side: SSSet,
    pub each_link_fixed: bool,
}

impl InputSets {
    pub fn never_update(&self, node: &Node) -> bool {
        self.each_link_fixed && !node.is_input() && node.pending_tick.is_none()
    }

    pub fn each(&self) -> impl Iterator<Item = InputStrengths> + '_ {
        self.default.strengths().flat_map(move |default| {
            self.side
                .strengths()
                .map(move |side| InputStrengths { default, side })
        })
    }
}

pub fn ss_sets(graph: &Graph) -> SSSets {
    let nodes: Vec<NodeIdx> = graph.node_indices().collect();
    let mut sets = vec![None; graph.node_bound()];
    let mut dirty = vec![true; graph.node_bound()];
    for &node_idx in &nodes {
        let node = &graph[node_idx];
        let initial = match node.is_input() {
            true => drivable(node),
            false => SSSet::singleton(node.state.power.get()),
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

            let inputs = collect_inputs(graph, &sets, node_idx);
            let node = &graph[node_idx];
            if inputs.never_update(node) {
                continue;
            }
            let old = sets[node_idx.index()].unwrap();
            let new = old.union(evaluate(node, &inputs));
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

fn collect_inputs(graph: &Graph, sets: &[Option<SSSet>], node_idx: NodeIdx) -> InputSets {
    let mut inputs = InputSets {
        default: SSSet::ZERO,
        side: SSSet::ZERO,
        each_link_fixed: true,
    };
    for edge in graph.edges(node_idx, Direction::Incoming) {
        let link = edge.weight();
        let carried = sets[edge.source().index()].unwrap().decay(link.weight);
        inputs.each_link_fixed &= carried.is_single();
        match link.kind {
            LinkKind::Default => inputs.default = inputs.default.combine(carried),
            LinkKind::Side => inputs.side = inputs.side.combine(carried),
        }
    }
    inputs
}

fn drivable(node: &Node) -> SSSet {
    let unpowered = InputStrengths::default();
    let accepted_inputs: Vec<AcceptedInput> = Input::all()
        .filter_map(|input| node.kind.accept(input))
        .collect();
    let mut reached = vec![node.state];
    let mut explored = 0;
    while let Some(&state) = reached.get(explored) {
        explored += 1;
        let driven = iproduct!(&accepted_inputs, Pending::ALL)
            .map(|(input, pending)| input.drive(state, pending));
        let ticked = node.can_tick().then(|| node.kind.tick(state, unpowered));
        for outcome in driven.chain(ticked) {
            if !reached.contains(&outcome.state) {
                reached.push(outcome.state);
            }
        }
    }
    reached
        .into_iter()
        .fold(SSSet::EMPTY, |set, state| set.with(state.power))
}

fn evaluate(node: &Node, inputs: &InputSets) -> SSSet {
    let mut reachable = SSSet::EMPTY;
    for inputs in inputs.each() {
        let updated = node.kind.update(node.state, inputs, Pending::None);
        reachable = reachable.union(strengths_while_inputs_hold(node.kind, updated, inputs));
        if node.pending_tick.is_some() {
            for state in [node.state, updated.state] {
                let ticked = node.kind.tick(state, inputs);
                reachable = reachable.union(strengths_while_inputs_hold(node.kind, ticked, inputs));
            }
        }
    }
    reachable
}

fn strengths_while_inputs_hold(
    kind: NodeKind,
    mut outcome: Outcome,
    inputs: InputStrengths,
) -> SSSet {
    let mut reached = SSSet::EMPTY.with(outcome.state.power);
    while outcome.schedule.is_some() {
        let ticked = kind.tick(outcome.state, inputs);
        if ticked.state == outcome.state {
            break;
        }
        reached = reached.with(ticked.state.power);
        outcome = ticked;
    }
    reached
}
