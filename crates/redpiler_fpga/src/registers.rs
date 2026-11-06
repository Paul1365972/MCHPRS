use crate::circuit::{initial_input_strength, Circuit, Kind, Link, StrengthSet, PULSE_PRIORITY};
use mchprs_redpiler::compile_graph::SignalStrength;
use mchprs_world::TickPriority;
use std::num::NonZeroU8;
use std::ops::RangeInclusive;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Field {
    Powered,
    Strength,
    /// Whether a comparator with two possible outputs outputs the higher one
    High {
        low: SignalStrength,
        high: SignalStrength,
    },
    Locked,
    Countdown(NonZeroU8),
    Pulsing,
}

/// What a read returns per node: its appearance for reports, or its full state for snapshots.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Contents {
    Appearance,
    State,
}

impl Field {
    pub(crate) fn width(self) -> usize {
        match self {
            Field::Strength => 4,
            Field::Countdown(delay) => usize::from(delay.get()),
            _ => 1,
        }
    }

    fn is_read_for(self, contents: Contents) -> bool {
        contents == Contents::State || !matches!(self, Field::Countdown(_) | Field::Pulsing)
    }
}

#[derive(Clone, Debug)]
pub(crate) struct NodeLayout {
    offset: usize,
    fields: Vec<Field>,
}

impl NodeLayout {
    pub(crate) fn field_offsets(&self) -> impl Iterator<Item = (Field, usize)> + '_ {
        self.fields.iter().scan(self.offset, |offset, &field| {
            let start = *offset;
            *offset += field.width();
            Some((field, start))
        })
    }

    pub(crate) fn read_fields(
        &self,
        contents: Contents,
    ) -> impl Iterator<Item = (Field, usize)> + '_ {
        self.field_offsets()
            .filter(move |(field, _)| field.is_read_for(contents))
    }

    fn width(&self, contents: Contents) -> usize {
        self.read_fields(contents)
            .map(|(field, _)| field.width())
            .sum()
    }
}

#[derive(Clone, Debug)]
pub(crate) struct ReadbackLayout {
    nodes: Vec<NodeLayout>,
}

impl ReadbackLayout {
    fn new(registers: &Registers) -> Self {
        let mut offset = 0;
        let nodes = (0..registers.circuit.nodes().len())
            .map(|index| {
                let layout = NodeLayout {
                    offset,
                    fields: registers.fields(index),
                };
                offset += layout.width(Contents::State);
                layout
            })
            .collect();
        Self { nodes }
    }

    pub(crate) fn node(&self, index: usize) -> &NodeLayout {
        &self.nodes[index]
    }

    pub(crate) fn bits(&self, contents: Contents, nodes: impl IntoIterator<Item = usize>) -> usize {
        nodes
            .into_iter()
            .map(|index| self.nodes[index].width(contents))
            .sum()
    }

    pub(crate) fn bytes(
        &self,
        contents: Contents,
        nodes: impl IntoIterator<Item = usize>,
    ) -> usize {
        self.bits(contents, nodes).div_ceil(8).max(1)
    }
}

pub(crate) struct Registers<'a> {
    pub(crate) circuit: &'a Circuit,
    strengths: Vec<StrengthSet>,
}

impl<'a> Registers<'a> {
    pub(crate) fn new(circuit: &'a Circuit) -> Self {
        Self {
            circuit,
            strengths: circuit.output_strengths(),
        }
    }

    pub(crate) fn strengths(&self, index: usize) -> StrengthSet {
        self.strengths[index]
    }

    pub(crate) fn literal(&self, index: usize) -> Option<SignalStrength> {
        self.strengths(index).single()
    }

    pub(crate) fn two_level(&self, index: usize) -> Option<(SignalStrength, SignalStrength)> {
        let strengths = self.strengths(index);
        if !matches!(self.circuit.nodes()[index].kind, Kind::Comparator { .. })
            || strengths.len() != 2
        {
            return None;
        }
        Some((strengths.minimum()?, strengths.maximum()?))
    }

    pub(crate) fn is_lockable(&self, index: usize) -> bool {
        let node = &self.circuit.nodes()[index];
        matches!(node.kind, Kind::Repeater { .. }) && !node.sides.is_empty()
    }

    /// Whether a repeater can never start a pulse, which it does by firing unpowered with its input
    /// off. One with delay one and a settled start fires unpowered only in the tick after its input
    /// turned on or it was unlocked with the input on. Each source fires at most once a tick, so
    /// the input cannot turn off again in time if its sources turn off no earlier than that tick's
    /// priority and no later than the sides that unlock it; levers, pressure plates and wires change
    /// at any time.
    pub(crate) fn is_pulse_free(&self, index: usize) -> bool {
        let nodes = self.circuit.nodes();
        let node = &nodes[index];
        let Kind::Repeater { delay: 1, .. } = node.kind else {
            return false;
        };
        let turn_offs = |links: &[Link]| -> Option<Vec<RangeInclusive<TickPriority>>> {
            links
                .iter()
                .map(|link| nodes[link.source].kind)
                .filter(|&kind| kind != Kind::Constant)
                .map(turn_off_priorities)
                .collect()
        };
        let (Some(inputs), Some(sides)) = (turn_offs(&node.inputs), turn_offs(&node.sides)) else {
            return false;
        };
        let initial = node.initial;
        let locked = !initial_input_strength(nodes, &node.sides).is_zero();
        let powered = !initial_input_strength(nodes, &node.inputs).is_zero();
        let settled = initial.pending.is_none()
            && initial.locked == locked
            && (locked || initial.is_powered() == powered);
        let unpowered_tick = node.kind.priority(false);
        let off_after_tick = inputs
            .iter()
            .all(|input| Some(*input.start()) >= unpowered_tick);
        let off_before_unlock = inputs
            .iter()
            .all(|input| sides.iter().all(|side| input.end() <= side.start()));
        settled && off_after_tick && off_before_unlock
    }

    pub(crate) fn readback_layout(&self) -> ReadbackLayout {
        ReadbackLayout::new(self)
    }

    fn fields(&self, index: usize) -> Vec<Field> {
        let kind = self.circuit.nodes()[index].kind;
        let mut fields = Vec::new();
        match kind {
            Kind::Constant => {}
            Kind::Comparator { .. } if let Some((low, high)) = self.two_level(index) => {
                fields.push(Field::High { low, high });
            }
            Kind::Comparator { .. } | Kind::Wire => {
                if self.literal(index).is_none() {
                    fields.push(Field::Strength);
                }
            }
            _ => fields.push(Field::Powered),
        }
        if self.is_lockable(index) {
            fields.push(Field::Locked);
        }
        if let Some(delay) = kind.delay() {
            fields.push(Field::Countdown(delay));
            if kind.tracks_pulsing() && !self.is_pulse_free(index) {
                fields.push(Field::Pulsing);
            }
        }
        fields
    }
}

/// The priorities from which to which a source can turn off, or none if it can change at any time
fn turn_off_priorities(kind: Kind) -> Option<RangeInclusive<TickPriority>> {
    if !matches!(
        kind,
        Kind::Torch | Kind::Repeater { .. } | Kind::Comparator { .. }
    ) {
        return None;
    }
    let first = kind.priority(true)?;
    let last = if matches!(kind, Kind::Repeater { .. }) {
        first.max(PULSE_PRIORITY)
    } else {
        first
    };
    Some(first..=last)
}
