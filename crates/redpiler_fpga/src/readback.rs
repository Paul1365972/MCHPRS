use crate::circuit::{Kind, PendingTick, State, PULSE_PRIORITY};
use crate::registers::{Field, ReadbackLayout, Registers};
use mchprs_redpiler::compile_graph::SignalStrength;
use std::num::NonZeroU8;

struct NodeReadback {
    kind: Kind,
    initial: State,
    literal: Option<SignalStrength>,
}

pub(crate) struct Readback {
    layout: ReadbackLayout,
    nodes: Vec<NodeReadback>,
}

struct Bits<'a> {
    bytes: &'a [u8],
    position: usize,
}

impl Bits<'_> {
    fn read(&mut self, width: usize) -> u32 {
        let value = (0..width).fold(0, |value, offset| {
            let position = self.position + offset;
            let bit = self.bytes[position / 8] >> (position % 8) & 1;
            value | u32::from(bit) << offset
        });
        self.position += width;
        value
    }
}

#[derive(Debug, thiserror::Error)]
pub(crate) enum ReadbackError {
    #[error("readback has {actual} bytes instead of {expected}")]
    Length { actual: usize, expected: usize },
    #[error("node {node}: countdown {value:#b} is not one-hot")]
    Countdown { node: usize, value: u32 },
    #[error("node {node}: more than one countdown is running")]
    Countdowns { node: usize },
}

impl Readback {
    pub(crate) fn new(registers: &Registers) -> Self {
        Self {
            layout: registers.readback_layout(),
            nodes: registers
                .circuit
                .nodes()
                .iter()
                .enumerate()
                .map(|(index, node)| NodeReadback {
                    kind: node.kind,
                    initial: node.initial,
                    literal: registers.literal(index),
                })
                .collect(),
        }
    }

    pub(crate) fn bytes(&self, nodes: impl IntoIterator<Item = usize>) -> usize {
        self.layout.bytes(nodes)
    }

    pub(crate) fn decode(
        &self,
        nodes: impl IntoIterator<Item = usize> + Clone,
        bytes: &[u8],
    ) -> Result<Vec<State>, ReadbackError> {
        let expected = self.bytes(nodes.clone());
        if bytes.len() != expected {
            return Err(ReadbackError::Length {
                actual: bytes.len(),
                expected,
            });
        }
        let mut bits = Bits { bytes, position: 0 };
        nodes
            .into_iter()
            .map(|index| self.decode_node(index, &mut bits))
            .collect()
    }

    fn decode_node(&self, index: usize, bits: &mut Bits) -> Result<State, ReadbackError> {
        let node = &self.nodes[index];
        let mut state = State {
            pending: None,
            ..node.initial
        };
        let mut countdown = None;
        for &field in &self.layout.node(index).fields {
            let value = bits.read(field.width());
            match field {
                Field::Powered => state.set_powered(value != 0),
                Field::High { low, high } => state.strength = if value != 0 { high } else { low },
                Field::Strength => {
                    state.strength =
                        SignalStrength::try_from(value).expect("a strength field has four bits");
                }
                Field::Locked => state.locked = value != 0,
                Field::Countdown(_) | Field::PulseCountdown(_) if value == 0 => {}
                Field::Countdown(_) | Field::PulseCountdown(_) => {
                    if value.count_ones() != 1 {
                        return Err(ReadbackError::Countdown { node: index, value });
                    }
                    if countdown.is_some() {
                        return Err(ReadbackError::Countdowns { node: index });
                    }
                    countdown = Some((value, matches!(field, Field::PulseCountdown(_))));
                }
            }
        }
        if let Some(strength) = node.literal {
            state.strength = strength;
        }
        if let Some((value, pulse)) = countdown {
            let priority = if pulse {
                PULSE_PRIORITY
            } else {
                node.kind
                    .priority(state.is_powered())
                    .expect("a countdown means a timed node")
            };
            state.pending = Some(PendingTick {
                delay: NonZeroU8::MIN.saturating_add(value.trailing_zeros() as u8),
                priority,
            });
        }
        Ok(state)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::Simulator;
    use crate::random_circuits::{input_nodes, random_circuit, Random};
    use crate::reference::Reference;

    fn countdown(state: State, has_pulse_countdown: bool, pulse_field: bool) -> u32 {
        match state.pending {
            Some(pending)
                if (has_pulse_countdown && pending.priority == PULSE_PRIORITY) == pulse_field =>
            {
                1 << (pending.delay.get() - 1)
            }
            _ => 0,
        }
    }

    fn encode(layout: &ReadbackLayout, states: &[State]) -> Vec<u8> {
        let mut bytes = vec![0; layout.bytes(0..states.len())];
        for (index, &state) in states.iter().enumerate() {
            let node = layout.node(index);
            let has_pulse_countdown = node
                .fields
                .iter()
                .any(|field| matches!(field, Field::PulseCountdown(_)));
            for (field, offset) in node.field_offsets() {
                let value = match field {
                    Field::Powered => u32::from(state.is_powered()),
                    Field::High { high, .. } => u32::from(state.strength == high),
                    Field::Strength => u32::from(state.strength.get()),
                    Field::Locked => u32::from(state.locked),
                    Field::Countdown(_) => countdown(state, has_pulse_countdown, false),
                    Field::PulseCountdown(_) => countdown(state, has_pulse_countdown, true),
                };
                for bit in 0..field.width() {
                    if value >> bit & 1 != 0 {
                        bytes[(offset + bit) / 8] |= 1 << ((offset + bit) % 8);
                    }
                }
            }
        }
        bytes
    }

    #[test]
    fn decoding_inverts_encoding() {
        for seed in 0..8 {
            let circuit = random_circuit(seed, 400);
            let registers = Registers::new(&circuit);
            let layout = registers.readback_layout();
            let readback = Readback::new(&registers);
            let mut reference = Reference::new(&circuit);
            let inputs = input_nodes(&circuit);
            let mut random = Random(seed ^ 0x6a09e667f3bcc909);
            for _ in 0..200 {
                if !inputs.is_empty() && random.chance(1, 3) {
                    let node = inputs[random.below(inputs.len())];
                    reference.input(node, random.chance(1, 2)).unwrap();
                }
                reference.advance(1).unwrap();
                let states = reference.states().unwrap();
                let bytes = encode(&layout, &states);
                assert_eq!(readback.decode(0..states.len(), &bytes).unwrap(), states);
            }
        }
    }

    #[test]
    fn decoding_rejects_a_wrong_length() {
        let circuit = random_circuit(0, 50);
        let readback = Readback::new(&Registers::new(&circuit));
        let expected = readback.bytes(0..50);
        assert!(matches!(
            readback.decode(0..50, &vec![0; expected + 1]),
            Err(ReadbackError::Length { actual, .. }) if actual == expected + 1
        ));
    }
}
