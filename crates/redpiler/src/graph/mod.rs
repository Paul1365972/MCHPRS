mod stable_graph;

use crate::ports::Role;
use mchprs_blocks::blocks::{ComparatorMode, Instrument};
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::BlockPos;
use mchprs_redstone::tick_queue::TickPriority;
// use petgraph::stable_graph::{NodeIndex, StableGraph};
use smallvec::SmallVec;
use stable_graph::{NodeIndex, StableGraph};

pub use stable_graph::{Direction, EdgeRef};
pub type NodeIdx = NodeIndex<u32>;
pub type Graph = StableGraph<Node, Link, u32>;

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum NodeKind {
    Repeater {
        delay: u8,
        prioritized: bool,
    },
    Torch,
    Comparator {
        mode: ComparatorMode,
        far_input: Option<SignalStrength>,
        prioritized: bool,
    },
    Lamp,
    Button,
    Lever,
    PressurePlate,
    Trapdoor,
    Wire,
    Constant,
    NoteBlock {
        instrument: Instrument,
        note: u8,
    },
}

impl NodeKind {
    pub fn conventional_role(&self) -> Option<Role> {
        match self {
            NodeKind::Button | NodeKind::Lever | NodeKind::PressurePlate => Some(Role::Input),
            NodeKind::Trapdoor | NodeKind::Lamp | NodeKind::NoteBlock { .. } => Some(Role::Output),
            _ => None,
        }
    }

    /// Whether the caller can own the output of this node.
    pub fn is_drivable(&self) -> bool {
        matches!(
            self,
            NodeKind::Button
                | NodeKind::Lever
                | NodeKind::PressurePlate
                | NodeKind::Repeater { .. }
                | NodeKind::Comparator { .. }
                | NodeKind::Torch
        )
    }

    pub fn is_binary_reader(&self) -> bool {
        !matches!(self, NodeKind::Comparator { .. } | NodeKind::Wire)
    }

    pub fn is_binary_source(&self) -> bool {
        !matches!(
            self,
            NodeKind::Comparator { .. } | NodeKind::Wire | NodeKind::Constant
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PendingTick {
    pub delay: u32,
    pub priority: TickPriority,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct QueuedTick {
    pub tick: PendingTick,
    /// Ticks due together with the same priority run in this order
    pub order: usize,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Hash)]
pub struct NodeState {
    pub power: SignalStrength,
    pub repeater_locked: bool,
    pub pending_tick: Option<QueuedTick>,
}

impl NodeState {
    pub fn from_power(power: SignalStrength) -> Self {
        Self {
            power,
            ..Default::default()
        }
    }

    pub fn from_powered(powered: bool) -> Self {
        Self::from_power(powered.into())
    }

    pub fn repeater(powered: bool, locked: bool) -> Self {
        Self {
            power: powered.into(),
            repeater_locked: locked,
            ..Default::default()
        }
    }

    pub fn is_powered(&self) -> bool {
        !self.power.is_zero()
    }
}

#[derive(Debug)]
pub struct Node {
    pub kind: NodeKind,
    pub block: SmallVec<[(BlockPos, u32); 1]>,
    pub name: Option<String>,
    pub state: NodeState,
    pub port: Option<Role>,
}

impl Node {
    pub fn is_input(&self) -> bool {
        self.port == Some(Role::Input)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum LinkKind {
    Default,
    Side,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Link {
    pub kind: LinkKind,
    pub weight: u8,
}

impl Link {
    pub fn new(kind: LinkKind, weight: u8) -> Self {
        Self { kind, weight }
    }

    pub fn default(weight: u8) -> Self {
        Self {
            kind: LinkKind::Default,
            weight,
        }
    }

    pub fn side(weight: u8) -> Self {
        Self {
            kind: LinkKind::Side,
            weight,
        }
    }
}
