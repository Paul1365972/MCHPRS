use super::{Circuit, ComparatorMode, Kind, Link};
use std::collections::VecDeque;

const OFF: u16 = 1;
const ON: u16 = 1 << 15;

fn attenuate(strengths: u16, attenuation: u8) -> u16 {
    (0..16)
        .filter(|strength| strengths & (1 << strength) != 0)
        .fold(0, |attenuated, strength: u8| {
            attenuated | 1 << strength.saturating_sub(attenuation)
        })
}

fn strongest(links: &[Link], strengths: &[u16]) -> u16 {
    let sets: Vec<_> = links
        .iter()
        .map(|link| attenuate(strengths[link.source], link.attenuation))
        .collect();
    let Some(floor) = sets.iter().map(|set| set.trailing_zeros()).max() else {
        return OFF;
    };
    sets.iter().fold(0, |union, set| union | set) & !((1 << floor) - 1)
}

fn comparator(mode: ComparatorMode, far_input: Option<u8>, rear: u16, side: u16) -> u16 {
    let rear = match far_input {
        Some(far) => {
            let below_full = if rear & !ON != 0 { 1 << far } else { 0 };
            below_full | rear & ON
        }
        None => rear,
    };
    let mut outputs = 0;
    for rear_strength in (0..16).filter(|strength| rear & (1 << strength) != 0) {
        for side_strength in (0..16).filter(|strength| side & (1 << strength) != 0) {
            let output = if rear_strength < side_strength {
                0
            } else if mode == ComparatorMode::Subtract {
                rear_strength - side_strength
            } else {
                rear_strength
            };
            outputs |= 1 << output;
        }
    }
    outputs
}

impl Circuit {
    pub(crate) fn output_strengths(&self) -> Vec<u16> {
        let mut strengths: Vec<u16> = self
            .nodes
            .iter()
            .map(|node| 1 << node.initial.strength)
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

    fn reachable_outputs(&self, index: usize, strengths: &[u16]) -> u16 {
        let node = &self.nodes[index];
        let input = strongest(&node.inputs, strengths);
        let can_be_on = input & !OFF != 0;
        let can_be_off = input & OFF != 0;
        let pending = node.kind.is_timed() && node.initial.delay != 0;
        let binary = |on: bool, off: bool| (if off { OFF } else { 0 }) | (if on { ON } else { 0 });
        match node.kind {
            Kind::Constant | Kind::Trapdoor => 0,
            Kind::Lever | Kind::Button | Kind::PressurePlate => OFF | ON,
            Kind::Torch => binary(can_be_off, can_be_on),
            Kind::Repeater { .. } => binary(can_be_on || pending, can_be_off || pending),
            Kind::Comparator {
                mode, far_input, ..
            } => comparator(mode, far_input, input, strongest(&node.sides, strengths)),
            Kind::Lamp => OFF,
            Kind::Wire => input,
        }
    }
}
