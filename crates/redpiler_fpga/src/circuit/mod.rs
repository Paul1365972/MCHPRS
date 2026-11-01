mod lower;
mod strengths;
mod validate;

pub(crate) use lower::priority;

use std::io;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ComparatorMode {
    Compare,
    Subtract,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Lever,
    Button,
    PressurePlate,
    Torch,
    Repeater {
        delay: u8,
        facing_diode: bool,
    },
    Comparator {
        mode: ComparatorMode,
        far_input: Option<u8>,
        facing_diode: bool,
    },
    Lamp,
    Trapdoor,
    Wire,
    Constant,
}

impl Kind {
    pub(crate) fn delay(self) -> Option<u8> {
        match self {
            Kind::Button => Some(10),
            Kind::Torch | Kind::Comparator { .. } => Some(1),
            Kind::Repeater { delay, .. } => Some(delay),
            Kind::Lamp => Some(2),
            _ => None,
        }
    }

    pub(crate) fn is_timed(self) -> bool {
        self.delay().is_some()
    }

    /// A repeater that does not face a diode schedules High when it will turn on and Higher when
    /// it will turn off, and its output only changes when it fires, so its pending priority
    /// follows its output.
    pub(crate) fn priority(self, powered: bool) -> Option<u8> {
        match self {
            Kind::Button | Kind::Torch | Kind::Lamp => Some(3),
            Kind::Comparator { facing_diode, .. } => Some(if facing_diode { 2 } else { 3 }),
            Kind::Repeater {
                facing_diode: true, ..
            } => Some(0),
            Kind::Repeater { .. } => Some(if powered { 1 } else { 2 }),
            _ => None,
        }
    }

    pub(crate) fn has_pulse_countdown(self) -> bool {
        matches!(
            self,
            Kind::Repeater {
                facing_diode: true,
                ..
            }
        )
    }

    pub(crate) fn has_boolean_power(self) -> bool {
        matches!(
            self,
            Kind::Lever
                | Kind::Button
                | Kind::PressurePlate
                | Kind::Torch
                | Kind::Repeater { .. }
                | Kind::Lamp
                | Kind::Trapdoor
        )
    }

    pub(crate) fn drives_links(self) -> bool {
        !matches!(self, Kind::Lamp | Kind::Trapdoor)
    }

    pub(crate) fn observes_inputs(self) -> bool {
        matches!(
            self,
            Kind::Torch
                | Kind::Repeater { .. }
                | Kind::Comparator { .. }
                | Kind::Lamp
                | Kind::Trapdoor
                | Kind::Wire
        )
    }
}

pub(crate) struct Snapshot {
    pub ticks: u64,
    pub states: Vec<State>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct State {
    pub strength: u8,
    pub locked: bool,
    pub delay: u8,
    pub priority: u8,
}

impl State {
    pub fn is_powered(self) -> bool {
        self.strength != 0
    }

    pub(crate) fn set_powered(&mut self, powered: bool) {
        self.strength = if powered { 15 } else { 0 };
    }
}

pub(crate) const PULSE_PRIORITY: u8 = 1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Link {
    pub source: usize,
    pub attenuation: u8,
}

#[derive(Clone, Debug)]
pub struct Node {
    pub kind: Kind,
    pub initial: State,
    pub inputs: Vec<Link>,
    pub sides: Vec<Link>,
    pub is_input: bool,
    pub is_output: bool,
}

#[derive(Clone, Debug)]
pub struct Circuit {
    pub(crate) nodes: Vec<Node>,
    pub(crate) identity: u64,
}

fn invalid(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, message.into())
}

impl Circuit {
    pub fn nodes(&self) -> &[Node] {
        &self.nodes
    }

    pub fn identity(&self) -> u64 {
        self.identity
    }

    pub(crate) fn io_nodes(&self) -> Vec<usize> {
        self.nodes
            .iter()
            .enumerate()
            .filter(|(_, node)| node.is_input || node.is_output)
            .map(|(index, _)| index)
            .collect()
    }

    pub fn new(nodes: Vec<Node>) -> io::Result<Self> {
        if nodes.is_empty() || u32::try_from(nodes.len()).is_err() {
            return Err(invalid("node count must fit the 32-bit protocol"));
        }
        let mut circuit = Self { nodes, identity: 0 };
        circuit.validate()?;
        circuit.identity = circuit.fingerprint();
        Ok(circuit)
    }

    fn fingerprint(&self) -> u64 {
        let mut hash = 0xcbf29ce484222325_u64;
        let mut feed = |bytes: &[u8]| {
            for byte in bytes {
                hash = (hash ^ u64::from(*byte)).wrapping_mul(0x100000001b3);
            }
        };
        feed(b"mchprs-fpga-phase-v3");
        feed(&(self.nodes.len() as u32).to_le_bytes());
        for node in &self.nodes {
            let tag = match node.kind {
                Kind::Lever => [0, 0, 0, 0],
                Kind::Button => [1, 0, 0, 0],
                Kind::PressurePlate => [2, 0, 0, 0],
                Kind::Torch => [3, 0, 0, 0],
                Kind::Repeater {
                    delay,
                    facing_diode,
                } => [4, delay, u8::from(facing_diode), 0],
                Kind::Comparator {
                    mode,
                    far_input,
                    facing_diode,
                } => [
                    5,
                    u8::from(mode == ComparatorMode::Subtract),
                    far_input.unwrap_or(16),
                    u8::from(facing_diode),
                ],
                Kind::Lamp => [6, 0, 0, 0],
                Kind::Trapdoor => [7, 0, 0, 0],
                Kind::Wire => [8, 0, 0, 0],
                Kind::Constant => [9, 0, 0, 0],
            };
            feed(&tag);
            let initial = node.initial;
            feed(&[
                initial.strength,
                u8::from(initial.locked),
                initial.delay,
                initial.priority,
            ]);
            feed(&[u8::from(node.is_input), u8::from(node.is_output)]);
            for links in [&node.inputs, &node.sides] {
                feed(&(links.len() as u64).to_le_bytes());
                for link in links {
                    feed(&(link.source as u32).to_le_bytes());
                    feed(&[link.attenuation]);
                }
            }
        }
        hash
    }
}
