mod error;
mod lower;
mod strength_set;
mod strengths;
mod validate;

pub(crate) use error::{CircuitError, NodeError};
pub(crate) use strength_set::StrengthSet;

use crate::screen::Screen;
use mchprs_redpiler::compile_graph::SignalStrength;
use mchprs_world::TickPriority;
use std::num::NonZeroU8;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ComparatorMode {
    Compare,
    Subtract,
}

impl ComparatorMode {
    pub(crate) fn output(
        self,
        far_input: Option<SignalStrength>,
        rear: SignalStrength,
        side: SignalStrength,
    ) -> SignalStrength {
        let rear = match far_input {
            Some(far_input) if rear != SignalStrength::Fifteen => far_input,
            _ => rear,
        };
        match self {
            ComparatorMode::Compare if rear >= side => rear,
            ComparatorMode::Compare => SignalStrength::Zero,
            ComparatorMode::Subtract => rear.saturating_sub(side.get()),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Kind {
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
        far_input: Option<SignalStrength>,
        facing_diode: bool,
    },
    Lamp,
    Trapdoor,
    Wire,
    Constant,
}

const LAMP_DELAY: NonZeroU8 = NonZeroU8::new(2).unwrap();
const BUTTON_DELAY: NonZeroU8 = NonZeroU8::new(10).unwrap();

impl Kind {
    pub(crate) fn delay(self) -> Option<NonZeroU8> {
        match self {
            Kind::Button => Some(BUTTON_DELAY),
            Kind::Torch | Kind::Comparator { .. } => Some(NonZeroU8::MIN),
            Kind::Repeater { delay, .. } => NonZeroU8::new(delay),
            Kind::Lamp => Some(LAMP_DELAY),
            _ => None,
        }
    }

    pub(crate) fn is_timed(self) -> bool {
        self.delay().is_some()
    }

    /// A repeater that does not face a diode schedules High when it will turn on and Higher when
    /// it will turn off, and its output only changes when it fires, so its pending priority
    /// follows its output.
    pub(crate) fn priority(self, powered: bool) -> Option<TickPriority> {
        match self {
            Kind::Button | Kind::Torch | Kind::Lamp => Some(TickPriority::Normal),
            Kind::Comparator { facing_diode, .. } => Some(if facing_diode {
                TickPriority::High
            } else {
                TickPriority::Normal
            }),
            Kind::Repeater {
                facing_diode: true, ..
            } => Some(TickPriority::Highest),
            Kind::Repeater { .. } => Some(if powered {
                TickPriority::Higher
            } else {
                TickPriority::High
            }),
            _ => None,
        }
    }

    pub(crate) fn scheduled_tick(self, powered: bool) -> Option<PendingTick> {
        Some(PendingTick {
            delay: self.delay()?,
            priority: self.priority(powered)?,
        })
    }

    pub(crate) fn is_input(self) -> bool {
        matches!(self, Kind::Lever | Kind::Button | Kind::PressurePlate)
    }

    /// A repeater facing a diode ends a pulse at a lower priority than it ticks at, so it records
    /// whether its pending tick ends a pulse.
    pub(crate) fn tracks_pulsing(self) -> bool {
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct PendingTick {
    pub(crate) delay: NonZeroU8,
    pub(crate) priority: TickPriority,
}

pub(crate) const PULSE_PRIORITY: TickPriority = TickPriority::Higher;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct State {
    pub(crate) strength: SignalStrength,
    pub(crate) locked: bool,
    pub(crate) pending: Option<PendingTick>,
}

impl State {
    pub(crate) fn is_powered(self) -> bool {
        !self.strength.is_zero()
    }

    pub(crate) fn set_powered(&mut self, powered: bool) {
        self.strength = powered.into();
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Link {
    pub(crate) source: usize,
    pub(crate) attenuation: u8,
}

impl Link {
    pub(crate) fn initial_strength(self, nodes: &[Node]) -> SignalStrength {
        nodes[self.source]
            .initial
            .strength
            .saturating_sub(self.attenuation)
    }
}

pub(crate) fn initial_input_strength(nodes: &[Node], links: &[Link]) -> SignalStrength {
    links
        .iter()
        .map(|link| link.initial_strength(nodes))
        .max()
        .unwrap_or_default()
}

#[derive(Clone, Debug)]
pub(crate) struct Node {
    pub(crate) kind: Kind,
    pub(crate) initial: State,
    pub(crate) inputs: Vec<Link>,
    pub(crate) sides: Vec<Link>,
    pub(crate) is_input: bool,
    pub(crate) is_output: bool,
}

const FINGERPRINT_VERSION: &[u8] = b"mchprs-fpga-phase-v3";
const FNV_OFFSET_BASIS: u64 = 0xcbf29ce484222325;
const FNV_PRIME: u64 = 0x100000001b3;
const NO_FAR_INPUT: u8 = 16;

#[derive(Clone, Debug)]
pub(crate) struct Circuit {
    nodes: Vec<Node>,
    io_nodes: Vec<usize>,
    screen: Screen,
    identity: u64,
}

impl Circuit {
    pub(crate) fn nodes(&self) -> &[Node] {
        &self.nodes
    }

    pub(crate) fn io_nodes(&self) -> &[usize] {
        &self.io_nodes
    }

    pub(crate) fn screen(&self) -> &Screen {
        &self.screen
    }

    pub(crate) fn identity(&self) -> u64 {
        self.identity
    }

    pub(crate) fn new(nodes: Vec<Node>, screen: Screen) -> Result<Self, CircuitError> {
        if nodes.is_empty() {
            return Err(CircuitError::Empty);
        }
        if u32::try_from(nodes.len()).is_err() {
            return Err(CircuitError::TooManyNodes(nodes.len()));
        }
        let io_nodes = nodes
            .iter()
            .enumerate()
            .filter(|(_, node)| node.is_input || node.is_output)
            .map(|(index, _)| index)
            .collect();
        let mut circuit = Self {
            nodes,
            io_nodes,
            screen,
            identity: 0,
        };
        circuit.validate()?;
        circuit.identity = circuit.fingerprint();
        Ok(circuit)
    }

    fn fingerprint(&self) -> u64 {
        let mut hash = FNV_OFFSET_BASIS;
        let mut feed = |bytes: &[u8]| {
            for byte in bytes {
                hash = (hash ^ u64::from(*byte)).wrapping_mul(FNV_PRIME);
            }
        };
        feed(FINGERPRINT_VERSION);
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
                    far_input.map_or(NO_FAR_INPUT, SignalStrength::get),
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
                initial.strength.get(),
                u8::from(initial.locked),
                initial.pending.map_or(0, |pending| pending.delay.get()),
                initial.pending.map_or(0, |pending| pending.priority as u8),
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
