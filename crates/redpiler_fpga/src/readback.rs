use crate::circuit::{Circuit, Kind, PendingTick, State, PULSE_PRIORITY};
use crate::registers::{readback_bytes, Field, Registers};
use mchprs_redpiler::compile_graph::SignalStrength;
use std::io;
use std::num::NonZeroU8;

struct NodeReadback {
    kind: Kind,
    initial: State,
    fields: Vec<Field>,
    literal: Option<SignalStrength>,
    two_level: Option<(SignalStrength, SignalStrength)>,
}

pub(crate) struct Readback {
    nodes: Vec<NodeReadback>,
}

struct Bits<'a> {
    bytes: &'a [u8],
    position: usize,
}

impl Bits<'_> {
    fn read(&mut self, width: u32) -> u32 {
        let value = (0..width).fold(0, |value, offset| {
            let position = self.position + offset as usize;
            let bit = self.bytes[position / 8] >> (position % 8) & 1;
            value | u32::from(bit) << offset
        });
        self.position += width as usize;
        value
    }
}

fn invalid(message: String) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

impl Readback {
    pub(crate) fn new(circuit: &Circuit) -> Self {
        let registers = Registers::new(circuit);
        Self {
            nodes: (0..circuit.nodes.len())
                .map(|index| NodeReadback {
                    kind: circuit.nodes[index].kind,
                    initial: circuit.nodes[index].initial,
                    fields: registers.fields(index),
                    literal: registers.literal(index),
                    two_level: registers.two_level(index),
                })
                .collect(),
        }
    }

    pub(crate) fn bytes(&self, nodes: impl IntoIterator<Item = usize>) -> usize {
        readback_bytes(
            nodes
                .into_iter()
                .flat_map(|index| &self.nodes[index].fields)
                .map(|field| field.width() as usize)
                .sum(),
        )
    }

    pub(crate) fn decode(
        &self,
        nodes: impl IntoIterator<Item = usize> + Clone,
        bytes: &[u8],
    ) -> io::Result<Vec<State>> {
        if bytes.len() != self.bytes(nodes.clone()) {
            return Err(invalid(format!(
                "readback has {} bytes instead of {}",
                bytes.len(),
                self.bytes(nodes)
            )));
        }
        let mut bits = Bits { bytes, position: 0 };
        nodes
            .into_iter()
            .map(|index| self.decode_node(index, &mut bits))
            .collect()
    }

    fn decode_node(&self, index: usize, bits: &mut Bits) -> io::Result<State> {
        let node = &self.nodes[index];
        let mut state = State {
            pending: None,
            ..node.initial
        };
        let mut countdowns = Vec::new();
        for &field in &node.fields {
            let value = bits.read(field.width());
            match field {
                Field::Powered => state.set_powered(value != 0),
                Field::High => {
                    let (low, high) = node.two_level.expect("a high bit means two levels");
                    state.strength = if value != 0 { high } else { low };
                }
                Field::Strength => {
                    state.strength =
                        SignalStrength::try_from(value).expect("a strength field has four bits");
                }
                Field::Locked => state.locked = value != 0,
                Field::Countdown(_) => countdowns.push((value, None)),
                Field::PulseCountdown(_) => countdowns.push((value, Some(PULSE_PRIORITY))),
            }
        }
        if let Some(strength) = node.literal {
            state.strength = strength;
        }
        countdowns.retain(|&(value, _)| value != 0);
        match countdowns[..] {
            [] => {}
            [(value, priority)] if value.count_ones() == 1 => {
                state.pending = Some(PendingTick {
                    delay: NonZeroU8::MIN.saturating_add(value.trailing_zeros() as u8),
                    priority: priority
                        .or(node.kind.priority(state.is_powered()))
                        .expect("a countdown means a timed node"),
                });
            }
            _ => {
                return Err(invalid(format!(
                    "node {index}: countdowns {countdowns:?} hold more than one pending tick"
                )));
            }
        }
        Ok(state)
    }
}
