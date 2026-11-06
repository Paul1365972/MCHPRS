use crate::circuit::{Kind, PendingTick, State, PULSE_PRIORITY};
use crate::registers::{Contents, Field, ReadbackLayout, Registers};
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
    #[error("node {node}: pulsing without a pending tick")]
    Pulsing { node: usize },
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

    pub(crate) fn bytes(
        &self,
        contents: Contents,
        nodes: impl IntoIterator<Item = usize>,
    ) -> usize {
        self.layout.bytes(contents, nodes)
    }

    pub(crate) fn decode(
        &self,
        contents: Contents,
        nodes: impl IntoIterator<Item = usize> + Clone,
        bytes: &[u8],
    ) -> Result<Vec<State>, ReadbackError> {
        let expected = self.bytes(contents, nodes.clone());
        if bytes.len() != expected {
            return Err(ReadbackError::Length {
                actual: bytes.len(),
                expected,
            });
        }
        let mut bits = Bits { bytes, position: 0 };
        nodes
            .into_iter()
            .map(|index| self.decode_node(contents, index, &mut bits))
            .collect()
    }

    fn decode_node(
        &self,
        contents: Contents,
        index: usize,
        bits: &mut Bits,
    ) -> Result<State, ReadbackError> {
        let node = &self.nodes[index];
        let mut state = State {
            pending: None,
            ..node.initial
        };
        let mut countdown = 0;
        let mut pulsing = false;
        for (field, _) in self.layout.node(index).read_fields(contents) {
            let value = bits.read(field.width());
            match field {
                Field::Powered => state.set_powered(value != 0),
                Field::Strength => {
                    state.strength =
                        SignalStrength::try_from(value).expect("a strength field has four bits");
                }
                Field::High { low, high } => state.strength = if value != 0 { high } else { low },
                Field::Locked => state.locked = value != 0,
                Field::Countdown(_) => countdown = value,
                Field::Pulsing => pulsing = value != 0,
            }
        }
        if let Some(strength) = node.literal {
            state.strength = strength;
        }
        if countdown == 0 {
            if pulsing {
                return Err(ReadbackError::Pulsing { node: index });
            }
            return Ok(state);
        }
        if countdown.count_ones() != 1 {
            return Err(ReadbackError::Countdown {
                node: index,
                value: countdown,
            });
        }
        let priority = if pulsing {
            PULSE_PRIORITY
        } else {
            node.kind
                .priority(state.is_powered())
                .expect("a countdown means a timed node")
        };
        state.pending = Some(PendingTick {
            delay: NonZeroU8::MIN.saturating_add(countdown.trailing_zeros() as u8),
            priority,
        });
        Ok(state)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::Simulator;
    use crate::random_circuits::{input_nodes, random_circuit, Random};
    use crate::reference::Reference;

    fn field_value(state: State, field: Field) -> u32 {
        match field {
            Field::Powered => u32::from(state.is_powered()),
            Field::Strength => u32::from(state.strength.get()),
            Field::High { high, .. } => u32::from(state.strength == high),
            Field::Locked => u32::from(state.locked),
            Field::Countdown(_) => state
                .pending
                .map_or(0, |pending| 1 << (pending.delay.get() - 1)),
            Field::Pulsing => u32::from(
                state
                    .pending
                    .is_some_and(|pending| pending.priority == PULSE_PRIORITY),
            ),
        }
    }

    fn encode(layout: &ReadbackLayout, contents: Contents, states: &[State]) -> Vec<u8> {
        let mut bytes = vec![0; layout.bytes(contents, 0..states.len())];
        let mut position = 0;
        for (index, &state) in states.iter().enumerate() {
            for (field, _) in layout.node(index).read_fields(contents) {
                let value = field_value(state, field);
                for bit in 0..field.width() {
                    bytes[position / 8] |= ((value >> bit & 1) as u8) << (position % 8);
                    position += 1;
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
                let bytes = encode(&layout, Contents::State, &states);
                let decoded = readback.decode(Contents::State, 0..states.len(), &bytes);
                assert_eq!(decoded.unwrap(), states);
                let bytes = encode(&layout, Contents::Appearance, &states);
                let decoded = readback.decode(Contents::Appearance, 0..states.len(), &bytes);
                let appearances: Vec<_> = states
                    .iter()
                    .map(|&state| State {
                        pending: None,
                        ..state
                    })
                    .collect();
                assert_eq!(decoded.unwrap(), appearances);
            }
        }
    }

    #[test]
    fn decoding_rejects_a_wrong_length() {
        let circuit = random_circuit(0, 50);
        let readback = Readback::new(&Registers::new(&circuit));
        let expected = readback.bytes(Contents::State, 0..50);
        assert!(matches!(
            readback.decode(Contents::State, 0..50, &vec![0; expected + 1]),
            Err(ReadbackError::Length { actual, .. }) if actual == expected + 1
        ));
    }
}
