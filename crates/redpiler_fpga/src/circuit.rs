mod lower;
mod strengths;

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
    pub(crate) fn is_timed(self) -> bool {
        matches!(
            self,
            Kind::Button
                | Kind::Torch
                | Kind::Repeater { .. }
                | Kind::Comparator { .. }
                | Kind::Lamp
        )
    }

    pub(crate) fn has_boolean_output(self) -> bool {
        matches!(
            self,
            Kind::Lever | Kind::Button | Kind::PressurePlate | Kind::Torch | Kind::Repeater { .. }
        )
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
    pub powered: bool,
    pub open: bool,
    pub locked: bool,
    pub delay: u8,
    pub priority: u8,
}

impl State {
    pub fn encode(self) -> u16 {
        assert!(self.strength <= 15 && self.delay <= 15 && self.priority <= 3);
        u16::from(self.strength)
            | (u16::from(self.powered) << 4)
            | (u16::from(self.locked) << 5)
            | (u16::from(self.delay) << 6)
            | (u16::from(self.priority) << 10)
            | (u16::from(self.open) << 12)
    }
}

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

    pub(crate) fn priorities(&self, index: usize) -> Vec<u8> {
        let node = &self.nodes[index];
        let mut used = [false; 4];
        match node.kind {
            Kind::Button | Kind::Torch | Kind::Lamp => used[3] = true,
            Kind::Comparator { facing_diode, .. } => used[if facing_diode { 2 } else { 3 }] = true,
            Kind::Repeater { facing_diode, .. } => {
                used[1] = true;
                used[if facing_diode { 0 } else { 2 }] = true;
            }
            _ => return Vec::new(),
        }
        if node.initial.delay != 0 {
            used[usize::from(node.initial.priority)] = true;
        }
        (0..4)
            .filter(|&priority| used[usize::from(priority)])
            .collect()
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

    fn validate(&self) -> io::Result<()> {
        for (index, node) in self.nodes.iter().enumerate() {
            if node.initial.strength > 15 || node.initial.delay > 15 || node.initial.priority > 3 {
                return Err(invalid(format!("node {index}: invalid state")));
            }
            if node.initial.open && node.kind != Kind::Trapdoor {
                return Err(invalid(format!(
                    "node {index}: only trapdoors can have an open state"
                )));
            }
            match node.kind {
                Kind::Repeater { delay, .. } if !(1..=15).contains(&delay) => {
                    return Err(invalid(format!(
                        "node {index}: repeater delay must be 1..15"
                    )))
                }
                Kind::Comparator {
                    far_input: Some(strength),
                    ..
                } if strength > 15 => {
                    return Err(invalid(format!(
                        "node {index}: comparator far input exceeds 15"
                    )))
                }
                _ => {}
            }
            for link in node.inputs.iter().chain(&node.sides) {
                let source = self
                    .nodes
                    .get(link.source)
                    .ok_or_else(|| invalid(format!("node {index}: link source is absent")))?;
                if link.attenuation > 15 {
                    return Err(invalid(format!("node {index}: attenuation exceeds 15")));
                }
                if source.kind == Kind::Wire {
                    return Err(invalid(format!(
                        "node {index}: wire source {} violates the frontend leaf-wire invariant",
                        link.source
                    )));
                }
            }
            if node.kind == Kind::Wire && !node.sides.is_empty() {
                return Err(invalid(format!(
                    "node {index}: wire cannot have side inputs"
                )));
            }
        }
        Ok(())
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
            feed(&node.initial.encode().to_le_bytes());
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
