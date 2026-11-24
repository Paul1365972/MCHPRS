use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::analysis::ss_sets::{ss_sets, SSSet};
use mchprs_redpiler::behavior::Pending;
use mchprs_redpiler::circuit::{inputs_in, Circuit, NodeId};
use mchprs_redpiler::graph::{LinkKind, NodeKind};
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
        let mut tick_priorities: Vec<_> = tick_priorities.collect();
        let ticks_at_start = |id: &NodeId| {
            let prioritized_repeater = NodeKind::Repeater {
                delay: 1,
                prioritized: true,
            };
            let node = circuit.node(*id);
            node.can_tick() && node.kind == prioritized_repeater
        };
        let mut unproven: Vec<NodeId> = circuit.ids().filter(ticks_at_start).collect();
        let mut pulse_free = vec![false; circuit.len()];
        // A repeater proven pulse-free changes only at the tick start, which can prove another.
        loop {
            let (proven, rest): (Vec<NodeId>, Vec<NodeId>) = unproven
                .into_iter()
                .partition(|&id| is_pulse_free(circuit, id, &tick_priorities));
            for &id in &proven {
                pulse_free[id.index()] = true;
                let pulse_end = repeater::PULSE_END_PRIORITY;
                tick_priorities[id.index()].retain(|&priority| priority != pulse_end);
            }
            unproven = rest;
            if proven.is_empty() {
                break;
            }
        }
        let firing = |id: NodeId| firing(circuit, id, &tick_priorities);
        CircuitAnalysis {
            strengths: circuit.ids().map(|id| sets.at(id.index())).collect(),
            pulse_free,
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

// A prioritized repeater of delay 1 pulses if its tick finds it off with its rear input off: a
// rear source fell after the tick was scheduled, in the same tick. A source changes at most once
// per tick, so it was on before, and the repeater was locked until a side source changed earlier
// in that tick. `no_rear_change_after_side` leaves no such tick.
fn is_pulse_free(circuit: &Circuit, id: NodeId, tick_priorities: &[Vec<TickPriority>]) -> bool {
    !is_rear_driven(circuit, id)
        && no_rear_change_after_side(circuit, id, tick_priorities)
        && starts_as_scheduled(circuit, id)
}

// For a torch or a comparator:
// In priority: the sources change no earlier than the node's priority, so whenever its output
// differs from what its inputs ask for, its tick is due.
// On a tick-start mismatch: the sources change only before the node's priority and at once, so
// its tick is due exactly when its output differed from its inputs at the tick start.
fn firing(circuit: &Circuit, id: NodeId, tick_priorities: &[Vec<TickPriority>]) -> Firing {
    let node = circuit.node(id);
    let own = match node.kind {
        _ if node.is_input() => return Firing::WhenDue,
        NodeKind::Torch => torch::PRIORITY,
        NodeKind::Comparator { prioritized, .. } => comparator::tick_priority(prioritized),
        NodeKind::Repeater {
            delay: 1,
            prioritized: false,
        } => return repeater_firing(circuit, id, tick_priorities),
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
    } else if changes_before_priority && change_at_once && starts_as_scheduled(circuit, id) {
        Firing::OnTickStartMismatch
    } else {
        Firing::WhenDue
    }
}

// An unprioritized repeater of delay 1 fires exactly when it was unlocked and differed from its
// rear input at the tick start, unless a scheduled tick loses its reason before the tick start
// and still acts. A tick that finds the repeater locked, or on with its rear input on, does
// nothing, which leaves three cases:
// - `no_rear_change_after_side`: no unlock schedules a tick to turn on before the rear input
//   falls, which would start a pulse.
// - `rear_stays`: a rear source that can turn on after a tick to turn off was scheduled is still
//   on when that tick fires.
// - `lock_stays`: a side source that can lock the repeater after a tick was scheduled still
//   locks it when that tick fires.
fn repeater_firing(circuit: &Circuit, id: NodeId, tick_priorities: &[Vec<TickPriority>]) -> Firing {
    let [off, on] = [false, true].map(|powered| repeater::tick_priority(false, powered));
    let (first, last) = (off.min(on), off.max(on));
    let changes = |kind: LinkKind| source_changes(circuit, id, kind, tick_priorities);
    let is_before = |source: &[TickPriority]| source.iter().all(|&change| change < first);
    let is_from = |priority: TickPriority, source: &[TickPriority]| {
        source.iter().all(|&change| priority <= change)
    };
    let rear_stays = changes(LinkKind::Default).all(|rear| is_before(rear) || is_from(first, rear));
    let lock_stays = changes(LinkKind::Side).all(|side| is_before(side) || is_from(last, side));
    let fires_on_mismatch = !is_rear_driven(circuit, id)
        && no_rear_change_after_side(circuit, id, tick_priorities)
        && rear_stays
        && lock_stays
        && starts_as_scheduled(circuit, id);
    match fires_on_mismatch {
        true => Firing::OnTickStartMismatch,
        false => Firing::WhenDue,
    }
}

// A driven rear source can change and change back between two ticks, which leaves a tick without
// its reason. A driven side source does no harm: the lock it leaves is the one the tick start
// sees.
fn is_rear_driven(circuit: &Circuit, repeater: NodeId) -> bool {
    sources(circuit, repeater, LinkKind::Default).any(|source| circuit.node(source).is_input())
}

fn no_rear_change_after_side(
    circuit: &Circuit,
    id: NodeId,
    tick_priorities: &[Vec<TickPriority>],
) -> bool {
    let changes = |kind: LinkKind| source_changes(circuit, id, kind, tick_priorities).flatten();
    changes(LinkKind::Side).all(|unlock| changes(LinkKind::Default).all(|change| change <= unlock))
}

fn source_changes<'a>(
    circuit: &'a Circuit,
    id: NodeId,
    kind: LinkKind,
    tick_priorities: &'a [Vec<TickPriority>],
) -> impl Iterator<Item = &'a Vec<TickPriority>> {
    sources(circuit, id, kind).map(|source| &tick_priorities[source.index()])
}

fn sources(circuit: &Circuit, id: NodeId, kind: LinkKind) -> impl Iterator<Item = NodeId> + '_ {
    let links = circuit.inputs(id).iter();
    let links = links.filter(move |link| link.kind == kind);
    links.map(|link| link.source)
}

fn starts_as_scheduled(circuit: &Circuit, id: NodeId) -> bool {
    let node = circuit.node(id);
    let Some(pending) = node.pending_tick else {
        return true;
    };
    let inputs = inputs_in(circuit, id);
    let scheduled = node.kind.update(node.state, inputs, Pending::None).schedule;
    scheduled == Some(pending.tick)
}
