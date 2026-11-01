use super::{Circuit, ComparatorMode, Kind, Link, StrengthSet};
use mchprs_redpiler::compile_graph::SignalStrength;
use std::collections::VecDeque;

fn strongest(links: &[Link], strengths: &[StrengthSet]) -> StrengthSet {
    let sets: Vec<_> = links
        .iter()
        .map(|link| strengths[link.source].attenuated(link.attenuation))
        .collect();
    let Some(floor) = sets.iter().filter_map(|set| set.minimum()).max() else {
        return StrengthSet::OFF;
    };
    sets.iter()
        .fold(StrengthSet::default(), |union, &set| union | set)
        .at_least(floor)
}

fn comparator(
    mode: ComparatorMode,
    far_input: Option<SignalStrength>,
    rear: StrengthSet,
    side: StrengthSet,
) -> StrengthSet {
    rear.iter()
        .flat_map(|rear| {
            side.iter()
                .map(move |side| mode.output(far_input, rear, side))
        })
        .collect()
}

fn binary(on: bool, off: bool) -> StrengthSet {
    let mut outputs = StrengthSet::default();
    if off {
        outputs |= StrengthSet::OFF;
    }
    if on {
        outputs |= StrengthSet::ON;
    }
    outputs
}

impl Circuit {
    pub(crate) fn output_strengths(&self) -> Vec<StrengthSet> {
        let mut strengths: Vec<_> = self
            .nodes
            .iter()
            .map(|node| StrengthSet::only(node.initial.strength))
            .collect();
        let mut consumers = vec![Vec::new(); self.nodes.len()];
        for (index, node) in self.nodes.iter().enumerate() {
            for link in node.inputs.iter().chain(&node.sides) {
                consumers[link.source].push(index);
            }
        }
        let mut queued = vec![true; self.nodes.len()];
        let mut queue: VecDeque<usize> = (0..self.nodes.len()).collect();
        while let Some(index) = queue.pop_front() {
            queued[index] = false;
            let grown = strengths[index] | self.reachable_outputs(index, &strengths);
            if grown == strengths[index] {
                continue;
            }
            strengths[index] = grown;
            for &consumer in &consumers[index] {
                if !queued[consumer] {
                    queued[consumer] = true;
                    queue.push_back(consumer);
                }
            }
        }
        strengths
    }

    fn reachable_outputs(&self, index: usize, strengths: &[StrengthSet]) -> StrengthSet {
        let node = &self.nodes[index];
        let input = strongest(&node.inputs, strengths);
        let can_be_on = input.maximum().is_some_and(|strength| !strength.is_zero());
        let can_be_off = input.contains(SignalStrength::Zero);
        let pending = node.initial.pending.is_some();
        match node.kind {
            Kind::Constant => StrengthSet::default(),
            Kind::Lever | Kind::Button | Kind::PressurePlate => StrengthSet::OFF | StrengthSet::ON,
            Kind::Torch => binary(can_be_off, can_be_on),
            Kind::Lamp | Kind::Trapdoor => binary(can_be_on, can_be_off),
            Kind::Repeater { .. } => binary(can_be_on || pending, can_be_off || pending),
            Kind::Comparator {
                mode, far_input, ..
            } => comparator(mode, far_input, input, strongest(&node.sides, strengths)),
            Kind::Wire => input,
        }
    }
}
