use crate::circuit::{Circuit, Kind, State};
use std::io;

pub(crate) struct Timer {
    pub(crate) delay: u8,
    pub(crate) priority: Option<u8>,
}

pub(crate) fn timer(kind: Kind) -> Option<Timer> {
    let (delay, priority) = match kind {
        Kind::Button => (10, Some(3)),
        Kind::Torch => (1, Some(3)),
        Kind::Repeater { delay, .. } => (delay, None),
        Kind::Comparator { facing_diode, .. } => (1, Some(if facing_diode { 2 } else { 3 })),
        Kind::Lamp => (2, Some(3)),
        _ => return None,
    };
    Some(Timer { delay, priority })
}

/// A countdown shift register: nodes with one possible priority, or whose priority follows their
/// output, count down in one shared lane, and other nodes count down in one lane per priority.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Lane {
    Shared,
    Priority(u8),
}

impl Lane {
    pub(crate) fn name(self, index: usize) -> String {
        match self {
            Lane::Shared => format!("{index}"),
            Lane::Priority(priority) => format!("{index}_{priority}"),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Field {
    Powered,
    Imported,
    High,
    Strength,
    Locked,
    Open,
    Cleared,
    Countdown(Lane, u32),
}

impl Field {
    pub(crate) fn width(self) -> u32 {
        match self {
            Field::Strength => 4,
            Field::Countdown(_, length) => length,
            _ => 1,
        }
    }

    pub(crate) fn register(self, index: usize) -> String {
        if let Field::Countdown(lane, _) = self {
            return format!("countdown_{}", lane.name(index));
        }
        let name = match self {
            Field::Powered => "powered",
            Field::Imported => "imported",
            Field::High => "high",
            Field::Strength => "strength",
            Field::Locked => "locked",
            Field::Open => "open",
            Field::Cleared => "cleared",
            Field::Countdown(..) => unreachable!("countdowns are named by lane"),
        };
        format!("{name}_{index}")
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

    pub(crate) fn imports_strength(&self, index: usize) -> bool {
        let initial = self.circuit.nodes[index].initial;
        initial.strength != if initial.powered { 15 } else { 0 }
    }

    pub(crate) fn has_lock(&self, index: usize) -> bool {
        let node = &self.circuit.nodes[index];
        matches!(node.kind, Kind::Repeater { .. })
            && (node.initial.locked || !node.sides.is_empty())
    }

    pub(crate) fn has_imported_flag(&self, index: usize) -> bool {
        let node = &self.circuit.nodes[index];
        match node.kind {
            kind if kind.has_boolean_output() => self.imports_strength(index),
            Kind::Comparator { .. } => node.initial.powered != (node.initial.strength != 0),
            _ => false,
        }
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

    pub(crate) fn has_open_register(&self, index: usize) -> bool {
        let node = &self.circuit.nodes[index];
        node.kind == Kind::Trapdoor && node.initial.open != node.initial.powered
    }

    /// A repeater that does not face a diode schedules High when it will turn on and Higher when
    /// it will turn off, and its output only changes when it fires, so its pending priority
    /// follows its output.
    pub(crate) fn priority_follows_output(&self, index: usize) -> bool {
        let node = &self.circuit.nodes[index];
        let follows = |priority: u8, powered: bool| priority == if powered { 1 } else { 2 };
        matches!(
            node.kind,
            Kind::Repeater {
                facing_diode: false,
                ..
            }
        ) && (node.initial.delay == 0 || follows(node.initial.priority, node.initial.powered))
    }

    pub(crate) fn lanes(&self, index: usize) -> Vec<Lane> {
        let priorities = self.circuit.priorities(index);
        if self.priority_follows_output(index) || priorities.len() == 1 {
            vec![Lane::Shared]
        } else {
            priorities.into_iter().map(Lane::Priority).collect()
        }
    }

    pub(crate) fn lane_priority(&self, index: usize, lane: Lane) -> Option<u8> {
        match lane {
            Lane::Priority(priority) => Some(priority),
            Lane::Shared if self.priority_follows_output(index) => None,
            Lane::Shared => Some(self.circuit.priorities(index)[0]),
        }
    }

    pub(crate) fn holds_initial_tick(&self, index: usize, lane: Lane) -> bool {
        let initial = self.circuit.nodes[index].initial;
        initial.delay != 0
            && match lane {
                Lane::Shared => true,
                Lane::Priority(priority) => priority == initial.priority,
            }
    }

    pub(crate) fn countdown_length(&self, index: usize, lane: Lane) -> u32 {
        let node = &self.circuit.nodes[index];
        let timer = timer(node.kind).expect("timed node");
        if self.holds_initial_tick(index, lane) {
            u32::from(timer.delay.max(node.initial.delay))
        } else {
            u32::from(timer.delay)
        }
    }

    pub(crate) fn fields(&self, index: usize) -> Vec<Field> {
        let node = &self.circuit.nodes[index];
        let mut fields = Vec::new();
        match node.kind {
            Kind::Constant => return fields,
            Kind::Comparator { .. } => {
                if self.two_level(index).is_some() {
                    fields.push(Field::High);
                } else if self.literal(index).is_none() {
                    fields.push(Field::Strength);
                }
            }
            Kind::Wire => {
                if self.literal(index).is_none() {
                    fields.push(Field::Strength);
                }
            }
            _ => fields.push(Field::Powered),
        }
        if self.has_imported_flag(index) {
            fields.push(Field::Imported);
        }
        if self.has_lock(index) {
            fields.push(Field::Locked);
        }
        if self.has_open_register(index) {
            fields.push(Field::Open);
        }
        if node.kind == Kind::Lamp && node.initial.strength != 0 {
            fields.push(Field::Cleared);
        }
        if timer(node.kind).is_some() {
            for lane in self.lanes(index) {
                fields.push(Field::Countdown(lane, self.countdown_length(index, lane)));
            }
        }
        fields
    }
}

pub(crate) fn readback_bytes(bits: usize) -> usize {
    bits.div_ceil(8).max(1)
}

struct NodeReadback {
    kind: Kind,
    initial: State,
    fields: Vec<Field>,
    literal: Option<u8>,
    two_level: Option<(u8, u8)>,
    priority_follows_output: bool,
    shared_priority: u8,
}

/// Packs the registers that can change, least significant bit first, so the hardware only
/// concatenates them and the hosts reconstruct each node's state.
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
                    priority_follows_output: registers.priority_follows_output(index),
                    shared_priority: circuit.priorities(index).first().copied().unwrap_or(0),
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
        let initial = node.initial;
        if node.kind == Kind::Constant {
            return Ok(initial);
        }
        let mut values = Values::default();
        for &field in &node.fields {
            let value = bits.read(field.width());
            match field {
                Field::Powered => values.powered = Some(value != 0),
                Field::Imported => values.imported = value != 0,
                Field::High => values.high = value != 0,
                Field::Strength => values.strength = value as u8,
                Field::Locked => values.locked = Some(value != 0),
                Field::Open => values.open = Some(value != 0),
                Field::Cleared => values.cleared = value != 0,
                Field::Countdown(lane, _) if value != 0 => values.pending.push((lane, value)),
                Field::Countdown(..) => {}
            }
        }
        let powered = values.powered.unwrap_or(initial.powered);
        let strength = match (node.literal, node.kind, node.two_level) {
            (Some(strength), _, _) => strength,
            (None, Kind::Comparator { .. }, Some((low, high))) => {
                if values.high {
                    high
                } else {
                    low
                }
            }
            (None, Kind::Comparator { .. } | Kind::Wire, _) => values.strength,
            (None, Kind::Lamp, _) if values.cleared => 0,
            (None, Kind::Lamp | Kind::Trapdoor, _) => initial.strength,
            (None, _, _) if values.imported => initial.strength,
            (None, _, _) if powered => 15,
            (None, _, _) => 0,
        };
        let mut state = State {
            strength,
            powered,
            open: false,
            locked: values.locked.unwrap_or(initial.locked),
            delay: 0,
            priority: 0,
        };
        match node.kind {
            Kind::Comparator { .. } if values.imported => state.powered = initial.powered,
            Kind::Comparator { .. } => state.powered = strength != 0,
            Kind::Wire => state.powered = initial.powered,
            Kind::Trapdoor => state.open = values.open.unwrap_or(powered),
            _ => {}
        }
        match values.pending[..] {
            [] => {}
            [(lane, remaining)] if remaining.count_ones() == 1 => {
                state.delay = remaining.trailing_zeros() as u8 + 1;
                state.priority = match lane {
                    Lane::Priority(priority) => priority,
                    Lane::Shared if node.priority_follows_output => {
                        if powered {
                            1
                        } else {
                            2
                        }
                    }
                    Lane::Shared => node.shared_priority,
                };
            }
            _ => {
                return Err(invalid(format!(
                    "node {index}: countdowns {:?} hold more than one pending tick",
                    values.pending
                )))
            }
        }
        Ok(state)
    }
}

#[derive(Default)]
struct Values {
    powered: Option<bool>,
    imported: bool,
    high: bool,
    strength: u8,
    locked: Option<bool>,
    open: Option<bool>,
    cleared: bool,
    pending: Vec<(Lane, u32)>,
}
