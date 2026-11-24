use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::analysis::ss_sets::{ss_sets, SSSet};
use mchprs_redpiler::behavior::{InputStrengths, Pending};
use mchprs_redpiler::circuit::{inputs_in, Circuit, NodeId};
use mchprs_redpiler::graph::{LinkKind, Node, NodeKind};
use mchprs_redstone::tick_queue::TickPriority;
use mchprs_redstone::{comparator, repeater, torch};
use std::collections::HashMap;

#[derive(Debug)]
pub struct CircuitAnalysis {
    strengths: Vec<SSSet>,
    pulse_free: Vec<bool>,
    firing: Vec<Firing>,
}

impl CircuitAnalysis {
    pub fn of(circuit: &Circuit) -> CircuitAnalysis {
        let sets = ss_sets(circuit);
        let mut priorities_by_kind: HashMap<NodeKind, Vec<TickPriority>> = HashMap::new();
        let tick_priorities = circuit.ids().map(|id| {
            let node = circuit.node(id);
            if !node.can_tick() {
                return Vec::new();
            }
            let kind = node.kind;
            let scheduled = || {
                let ticks = kind.schedulable_ticks();
                ticks.iter().map(|tick| tick.priority).collect()
            };
            let priorities = priorities_by_kind.entry(kind);
            priorities.or_insert_with(scheduled).clone()
        });
        let tick_priorities: Vec<_> = tick_priorities.collect();
        let is_pulse_free =
            |id: NodeId| is_pulse_free(circuit, id, inputs_in(circuit, id), &tick_priorities);
        let firing = |id: NodeId| firing(circuit, id, inputs_in(circuit, id), &tick_priorities);
        CircuitAnalysis {
            strengths: circuit.ids().map(|id| sets.at(id.index())).collect(),
            pulse_free: circuit.ids().map(is_pulse_free).collect(),
            firing: circuit.ids().map(firing).collect(),
        }
    }

    pub fn strengths(&self, node: NodeId) -> SSSet {
        self.strengths[node.index()]
    }

    pub fn binary_strengths(&self, node: NodeId) -> Option<(SignalStrength, SignalStrength)> {
        match strengths_of(self.strengths(node)).as_slice() {
            &[low, high] => Some((low, high)),
            _ => None,
        }
    }

    pub fn is_pulse_free(&self, repeater: NodeId) -> bool {
        self.pulse_free[repeater.index()]
    }

    pub fn firing(&self, node: NodeId) -> Firing {
        self.firing[node.index()]
    }
}

/// When a node fires, numbered as `pkg::FIRES_*`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Firing {
    WhenDue = 0,
    InPriority = 1,
    OnTickStartMismatch = 2,
}

pub(crate) fn strengths_of(set: SSSet) -> Vec<SignalStrength> {
    SignalStrength::all()
        .filter(|strength| set.contains(strength.get()))
        .collect()
}

// A pulse-free repeater's rear input remains different from its output until its tick fires.
fn is_pulse_free(
    circuit: &Circuit,
    id: NodeId,
    inputs: InputStrengths,
    tick_priorities: &[Vec<TickPriority>],
) -> bool {
    let node = circuit.node(id);
    let NodeKind::Repeater {
        delay: 1,
        prioritized,
    } = node.kind
    else {
        return false;
    };
    let sources = |kind: LinkKind| {
        let links = circuit.inputs(id).iter();
        let links = links.filter(move |link| link.kind == kind);
        links.map(|link| link.source)
    };
    let priorities = |source: NodeId| tick_priorities[source.index()].iter().copied();
    let defaults: Vec<NodeId> = sources(LinkKind::Default).collect();
    let is_driven = defaults
        .iter()
        .any(|&source| circuit.node(source).is_input());
    let changing = defaults
        .iter()
        .filter(|&&source| priorities(source).next().is_some());
    let changes: Vec<TickPriority> = defaults
        .iter()
        .flat_map(|&source| priorities(source))
        .collect();
    let change_at_once =
        changing.count() <= 1 || changes.iter().all(|&change| change == changes[0]);
    let own_ticks = [false, true].map(|powered| repeater::tick_priority(prioritized, powered));
    let own_tick_first = own_ticks
        .iter()
        .all(|own| changes.iter().all(|change| own <= change));
    let mut unlocks = sources(LinkKind::Side).flat_map(priorities);
    let unlock_last = unlocks.all(|unlock| changes.iter().all(|&change| change <= unlock));
    let starts_as_scheduled = starts_as_scheduled(node, inputs);
    !is_driven && change_at_once && own_tick_first && unlock_last && starts_as_scheduled
}

// In priority: the sources change no earlier than the node's priority, so whenever its output
// differs from what its inputs ask for, its tick is due.
// On a tick-start mismatch: the sources change only before the node's priority and at once, so
// its tick is due exactly when its output differed from its inputs at the tick start.
fn firing(
    circuit: &Circuit,
    id: NodeId,
    inputs: InputStrengths,
    tick_priorities: &[Vec<TickPriority>],
) -> Firing {
    let node = circuit.node(id);
    let own = match node.kind {
        _ if node.is_input() => return Firing::WhenDue,
        NodeKind::Torch => torch::PRIORITY,
        NodeKind::Comparator { prioritized, .. } => comparator::tick_priority(prioritized),
        _ => return Firing::WhenDue,
    };
    let mut sources: Vec<NodeId> = circuit.inputs(id).iter().map(|link| link.source).collect();
    sources.sort();
    sources.dedup();
    let is_driven = sources
        .iter()
        .any(|&source| circuit.node(source).is_input());
    let changes: Vec<&[TickPriority]> = sources
        .iter()
        .map(|source| tick_priorities[source.index()].as_slice())
        .filter(|changes| !changes.is_empty())
        .collect();
    let all_changes = changes.concat();
    let change_at_once =
        changes.len() <= 1 || all_changes.iter().all(|&change| change == all_changes[0]);
    let changes_in_or_after_priority = all_changes.iter().all(|&change| own <= change);
    let changes_before_priority = all_changes.iter().all(|&change| change < own);
    if is_driven {
        Firing::WhenDue
    } else if changes_in_or_after_priority {
        Firing::InPriority
    } else if changes_before_priority && change_at_once && starts_as_scheduled(node, inputs) {
        Firing::OnTickStartMismatch
    } else {
        Firing::WhenDue
    }
}

fn starts_as_scheduled(node: &Node, inputs: InputStrengths) -> bool {
    let pending = node.pending_tick.map(|queued| queued.tick);
    let scheduled = node.kind.update(node.state, inputs, Pending::None).schedule;
    pending.is_none() || pending == scheduled
}
