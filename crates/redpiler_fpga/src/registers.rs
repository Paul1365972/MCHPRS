use crate::circuit::{Circuit, Kind, StrengthSet};
use mchprs_redpiler::compile_graph::SignalStrength;
use std::num::NonZeroU8;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Field {
    Powered,
    High {
        low: SignalStrength,
        high: SignalStrength,
    },
    Strength,
    Locked,
    Countdown(NonZeroU8),
    PulseCountdown(NonZeroU8),
}

impl Field {
    pub(crate) fn width(self) -> usize {
        match self {
            Field::Strength => 4,
            Field::Countdown(delay) | Field::PulseCountdown(delay) => usize::from(delay.get()),
            _ => 1,
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) struct NodeLayout {
    pub(crate) offset: usize,
    pub(crate) width: usize,
    pub(crate) fields: Vec<Field>,
}

impl NodeLayout {
    pub(crate) fn field_offsets(&self) -> impl Iterator<Item = (Field, usize)> {
        self.fields.iter().scan(self.offset, |offset, &field| {
            let start = *offset;
            *offset += field.width();
            Some((field, start))
        })
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
                let fields = registers.fields(index);
                let width = fields.iter().map(|field| field.width()).sum();
                let layout = NodeLayout {
                    offset,
                    width,
                    fields,
                };
                offset += width;
                layout
            })
            .collect();
        Self { nodes }
    }

    pub(crate) fn node(&self, index: usize) -> &NodeLayout {
        &self.nodes[index]
    }

    pub(crate) fn bits(&self, nodes: impl IntoIterator<Item = usize>) -> usize {
        nodes.into_iter().map(|index| self.nodes[index].width).sum()
    }

    pub(crate) fn bytes(&self, nodes: impl IntoIterator<Item = usize>) -> usize {
        self.bits(nodes).div_ceil(8).max(1)
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
            if kind.has_pulse_countdown() {
                fields.push(Field::PulseCountdown(delay));
            }
        }
        fields
    }
}
