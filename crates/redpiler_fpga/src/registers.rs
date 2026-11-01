use crate::circuit::{Circuit, Kind};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Field {
    Powered,
    High,
    Strength,
    Locked,
    Countdown(u8),
    PulseCountdown(u8),
}

impl Field {
    pub(crate) fn width(self) -> u32 {
        match self {
            Field::Strength => 4,
            Field::Countdown(delay) | Field::PulseCountdown(delay) => u32::from(delay),
            _ => 1,
        }
    }
}

pub(crate) struct Registers<'a> {
    pub(crate) circuit: &'a Circuit,
    strengths: Vec<u16>,
}

impl<'a> Registers<'a> {
    pub(crate) fn new(circuit: &'a Circuit) -> Self {
        Self {
            circuit,
            strengths: circuit.output_strengths(),
        }
    }

    pub(crate) fn strengths(&self, index: usize) -> u16 {
        self.strengths[index]
    }

    pub(crate) fn literal(&self, index: usize) -> Option<u8> {
        let strengths = self.strengths(index);
        (strengths.count_ones() == 1).then(|| strengths.trailing_zeros() as u8)
    }

    pub(crate) fn two_level(&self, index: usize) -> Option<(u8, u8)> {
        let strengths = self.strengths(index);
        (matches!(self.circuit.nodes[index].kind, Kind::Comparator { .. })
            && strengths.count_ones() == 2)
            .then(|| {
                (
                    strengths.trailing_zeros() as u8,
                    15 - strengths.leading_zeros() as u8,
                )
            })
    }

    pub(crate) fn is_lockable(&self, index: usize) -> bool {
        let node = &self.circuit.nodes[index];
        matches!(node.kind, Kind::Repeater { .. }) && !node.sides.is_empty()
    }

    pub(crate) fn fields(&self, index: usize) -> Vec<Field> {
        let kind = self.circuit.nodes[index].kind;
        let mut fields = Vec::new();
        match kind {
            Kind::Constant => {}
            Kind::Comparator { .. } if self.two_level(index).is_some() => {
                fields.push(Field::High);
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

pub(crate) fn readback_bytes(bits: usize) -> usize {
    bits.div_ceil(8).max(1)
}
