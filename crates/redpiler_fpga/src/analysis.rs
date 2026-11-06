use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::analysis::ss_sets::{ss_sets, SSSet};
use mchprs_redpiler::behavior::{InputStrengths, Pending};
use mchprs_redpiler::circuit::{inputs_in, Circuit, NodeId};
use mchprs_redpiler::graph::{Graph, Link, LinkKind, NodeIdx, NodeKind};
use mchprs_redstone::repeater;
use mchprs_redstone::tick_queue::TickPriority;
use std::collections::HashMap;

#[derive(Debug)]
pub struct CircuitAnalysis {
    strengths: Vec<SSSet>,
    pulse_free: Vec<bool>,
}

impl CircuitAnalysis {
    pub fn of(circuit: &Circuit) -> CircuitAnalysis {
        let mut graph = Graph::default();
        let nodes = circuit.ids().map(|id| circuit.node(id).clone());
        let indices: Vec<NodeIdx> = nodes.map(|node| graph.add_node(node)).collect();
        let index = |id: NodeId| indices[id.index()];
        for link in circuit.ids().flat_map(|id| circuit.outputs(id)) {
            let weights = Link::new(link.kind, link.weight);
            graph.add_link(index(link.source), index(link.target), weights);
        }
        let sets = ss_sets(&graph);
        let mut priorities_by_kind: HashMap<NodeKind, Vec<TickPriority>> = HashMap::new();
        let tick_priorities = circuit.ids().map(|id| {
            let kind = circuit.node(id).kind;
            let scheduled = || {
                let ticks = kind.schedulable_ticks();
                ticks.iter().map(|tick| tick.priority).collect()
            };
            let priorities = priorities_by_kind.entry(kind);
            priorities.or_insert_with(scheduled).clone()
        });
        let tick_priorities: Vec<_> = tick_priorities.collect();
        let is_pulse_free =
            |id: NodeId| is_pulse_free(circuit, id, inputs_in(&graph, index(id)), &tick_priorities);
        CircuitAnalysis {
            strengths: circuit.ids().map(|id| sets.of(index(id))).collect(),
            pulse_free: circuit.ids().map(is_pulse_free).collect(),
        }
    }

    pub fn strengths(&self, node: NodeId) -> SSSet {
        self.strengths[node.index()]
    }

    /// The output that the inputs of such a comparator give is always one of the two strengths.
    pub fn binary_strengths(&self, node: NodeId) -> Option<(SignalStrength, SignalStrength)> {
        match strengths_of(self.strengths(node)).as_slice() {
            &[low, high] => Some((low, high)),
            _ => None,
        }
    }

    /// While such a repeater has a pending tick, its input differs from its output.
    pub fn is_pulse_free(&self, repeater: NodeId) -> bool {
        self.pulse_free[repeater.index()]
    }
}

pub(crate) fn strengths_of(set: SSSet) -> Vec<SignalStrength> {
    SignalStrength::all()
        .filter(|strength| set.contains(strength.get()))
        .collect()
}

/// A tick of a repeater with a delay of 1 is scheduled when its input differs from its output, and
/// it runs in the next redstone tick. The input stays as it is until then if no default source
/// - is driven, which can happen any number of times between two redstone ticks,
/// - ticks later in the redstone tick that schedules: a node ticks once in a redstone tick, so
///   either only one source ticks at all or all tick with one priority, and none ticks after a
///   side source, whose tick can unlock the repeater,
/// - ticks in the next redstone tick before the repeater does.
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
    let ticks = |source: NodeId| tick_priorities[source.index()].iter().copied();
    let mut default = sources(LinkKind::Default);
    let is_driven = default.any(|source| circuit.node(source).is_input());
    let ticking =
        sources(LinkKind::Default).filter(|&source| !tick_priorities[source.index()].is_empty());
    let default_ticks: Vec<_> = sources(LinkKind::Default).flat_map(ticks).collect();
    let all_at_once = default_ticks.iter().all(|tick| *tick == default_ticks[0]);
    let own_ticks = [false, true].map(|powered| repeater::tick_priority(prioritized, powered));
    let none_before_own = own_ticks
        .iter()
        .all(|own| default_ticks.iter().all(|tick| own <= tick));
    let mut unlocks = sources(LinkKind::Side).flat_map(ticks);
    let none_after_unlock = unlocks.all(|unlock| default_ticks.iter().all(|tick| *tick <= unlock));
    let pending = node.pending_tick.map(|queued| queued.tick);
    let scheduled = node.kind.update(node.state, inputs, Pending::None).schedule;
    !is_driven
        && (ticking.count() <= 1 || all_at_once)
        && none_before_own
        && none_after_unlock
        && (pending.is_none() || pending == scheduled)
}
