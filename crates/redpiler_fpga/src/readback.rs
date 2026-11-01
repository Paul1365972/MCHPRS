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
