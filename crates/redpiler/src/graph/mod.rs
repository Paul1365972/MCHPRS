mod stable_graph;

use crate::ports::Role;
use mchprs_blocks::blocks::{ComparatorMode, Instrument};
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::BlockPos;
use mchprs_redstone::tick_queue::TickPriority;
// use petgraph::stable_graph::{NodeIndex, StableGraph};
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;
use stable_graph::{NodeIndex, StableGraph};

pub use stable_graph::{Direction, EdgeRef};
pub type NodeIdx = NodeIndex<u32>;
pub type Graph = StableGraph<Node, Link, u32>;

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
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
            NodeKind::Repeater { .. }
            | NodeKind::Torch
            | NodeKind::Comparator { .. }
            | NodeKind::Wire
            | NodeKind::Constant => None,
        }
    }

    pub fn drive_kind(&self) -> Option<DriveKind> {
        match self {
            NodeKind::Lever
            | NodeKind::PressurePlate
            | NodeKind::Repeater { .. }
            | NodeKind::Torch => Some(DriveKind::OnOff),
            NodeKind::Comparator { .. } => Some(DriveKind::Strength),
            NodeKind::Button => Some(DriveKind::Press),
            NodeKind::Lamp
            | NodeKind::Trapdoor
            | NodeKind::Wire
            | NodeKind::Constant
            | NodeKind::NoteBlock { .. } => None,
        }
    }

    pub fn is_binary_reader(&self) -> bool {
        match self {
            NodeKind::Comparator { .. } | NodeKind::Wire => false,
            NodeKind::Repeater { .. }
            | NodeKind::Torch
            | NodeKind::Lamp
            | NodeKind::Button
            | NodeKind::Lever
            | NodeKind::PressurePlate
            | NodeKind::Trapdoor
            | NodeKind::Constant
            | NodeKind::NoteBlock { .. } => true,
        }
    }

    pub fn is_binary_source(&self) -> bool {
        match self {
            NodeKind::Comparator { .. } | NodeKind::Wire | NodeKind::Constant => false,
            NodeKind::Repeater { .. }
            | NodeKind::Torch
            | NodeKind::Lamp
            | NodeKind::Button
            | NodeKind::Lever
            | NodeKind::PressurePlate
            | NodeKind::Trapdoor
            | NodeKind::NoteBlock { .. } => true,
        }
    }

    pub fn is_link_source(&self) -> bool {
        match self {
            NodeKind::Lamp | NodeKind::Trapdoor | NodeKind::Wire | NodeKind::NoteBlock { .. } => {
                false
            }
            NodeKind::Repeater { .. }
            | NodeKind::Torch
            | NodeKind::Comparator { .. }
            | NodeKind::Button
            | NodeKind::Lever
            | NodeKind::PressurePlate
            | NodeKind::Constant => true,
        }
    }

    pub fn is_link_target(&self) -> bool {
        match self {
            NodeKind::Button | NodeKind::Lever | NodeKind::PressurePlate | NodeKind::Constant => {
                false
            }
            NodeKind::Repeater { .. }
            | NodeKind::Torch
            | NodeKind::Comparator { .. }
            | NodeKind::Lamp
            | NodeKind::Trapdoor
            | NodeKind::Wire
            | NodeKind::NoteBlock { .. } => true,
        }
    }

    pub fn is_tickable(&self) -> bool {
        match self {
            NodeKind::Repeater { .. }
            | NodeKind::Torch
            | NodeKind::Comparator { .. }
            | NodeKind::Lamp
            | NodeKind::Button => true,
            NodeKind::Lever
            | NodeKind::PressurePlate
            | NodeKind::Trapdoor
            | NodeKind::Wire
            | NodeKind::Constant
            | NodeKind::NoteBlock { .. } => false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveKind {
    OnOff,
    Strength,
    Press,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PendingTick {
    pub delay: u32,
    pub priority: TickPriority,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct QueuedTick {
    pub tick: PendingTick,
    /// Ticks due together with the same priority run in this order
    pub order: usize,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct NodeState {
    pub power: SignalStrength,
    pub repeater_locked: bool,
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
        }
    }

    pub fn is_powered(&self) -> bool {
        !self.power.is_zero()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Node {
    pub kind: NodeKind,
    pub blocks: SmallVec<[(BlockPos, u32); 1]>,
    pub name: Option<String>,
    pub state: NodeState,
    pub pending_tick: Option<QueuedTick>,
    pub port: Option<Role>,
    pub observed: bool,
}

impl Node {
    pub fn is_input(&self) -> bool {
        self.port == Some(Role::Input)
    }

    pub fn can_tick(&self) -> bool {
        self.kind.is_tickable() && (!self.is_input() || self.kind == NodeKind::Button)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
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
