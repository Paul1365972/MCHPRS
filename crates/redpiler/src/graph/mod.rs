mod stable_graph;

use mchprs_blocks::blocks::{ComparatorMode, Instrument};
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::BlockPos;
use mchprs_redstone::tick_queue::PendingTick;
use mchprs_simulation::ports::Role;
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;
use stable_graph::{NodeIndex, StableGraph};

pub use stable_graph::{Direction, EdgeRef};
pub type NodeIdx = NodeIndex;
pub type Graph = StableGraph<Node, Link>;

impl Graph {
    pub fn add_link(&mut self, source: NodeIdx, target: NodeIdx, link: Link) {
        let parallel = self
            .edges(target, Direction::Incoming)
            .find(|edge| edge.source() == source && edge.weight().kind == link.kind)
            .map(|edge| edge.id());
        match parallel {
            Some(parallel) => {
                let weight = &mut self.edge_mut(parallel).weight;
                *weight = link.weight.min(*weight);
            }
            None => {
                self.add_edge(source, target, link);
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
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
    Button {
        press_ticks: u8,
    },
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
    pub fn traits(self) -> Traits {
        const ON_OFF: Option<SignalKind> = Some(SignalKind::OnOff);
        const STRENGTH: Option<SignalKind> = Some(SignalKind::Strength);
        let plain = Traits {
            role: None,
            drive: None,
            reads: None,
            emits: None,
            side_inputs: false,
            lockable: false,
            ticks: false,
        };
        match self {
            NodeKind::Repeater { .. } => Traits {
                drive: Some(DriveKind::OnOff),
                reads: ON_OFF,
                emits: ON_OFF,
                side_inputs: true,
                lockable: true,
                ticks: true,
                ..plain
            },
            NodeKind::Torch => Traits {
                drive: Some(DriveKind::OnOff),
                reads: ON_OFF,
                emits: ON_OFF,
                ticks: true,
                ..plain
            },
            NodeKind::Comparator { .. } => Traits {
                drive: Some(DriveKind::Strength),
                reads: STRENGTH,
                emits: STRENGTH,
                side_inputs: true,
                ticks: true,
                ..plain
            },
            NodeKind::Lamp => Traits {
                role: Some(Role::Output),
                reads: ON_OFF,
                ticks: true,
                ..plain
            },
            NodeKind::Button { .. } => Traits {
                role: Some(Role::Input),
                drive: Some(DriveKind::Press),
                emits: ON_OFF,
                ticks: true,
                ..plain
            },
            NodeKind::Lever => Traits {
                role: Some(Role::Input),
                drive: Some(DriveKind::OnOff),
                emits: ON_OFF,
                ..plain
            },
            NodeKind::PressurePlate => Traits {
                role: Some(Role::Input),
                drive: Some(DriveKind::Touch),
                emits: ON_OFF,
                ticks: true,
                ..plain
            },
            NodeKind::Trapdoor | NodeKind::NoteBlock { .. } => Traits {
                role: Some(Role::Output),
                reads: ON_OFF,
                ..plain
            },
            NodeKind::Wire => Traits {
                reads: STRENGTH,
                ..plain
            },
            NodeKind::Constant => Traits {
                emits: STRENGTH,
                lockable: true,
                ..plain
            },
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Traits {
    pub role: Option<Role>,
    pub drive: Option<DriveKind>,
    pub reads: Option<SignalKind>,
    pub emits: Option<SignalKind>,
    pub side_inputs: bool,
    pub lockable: bool,
    pub ticks: bool,
}

impl Traits {
    pub fn is_on_off(&self) -> bool {
        [self.reads, self.emits].contains(&Some(SignalKind::OnOff))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SignalKind {
    OnOff,
    Strength,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveKind {
    OnOff,
    Strength,
    Press,
    Touch,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct QueuedTick {
    pub tick: PendingTick,
    pub order: usize,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct NodeState {
    pub power: SignalStrength,
    pub repeater_locked: bool,
    pub plate_touched: bool,
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

    pub fn shown(self) -> NodeState {
        NodeState {
            plate_touched: false,
            ..self
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Node {
    pub kind: NodeKind,
    pub blocks: SmallVec<[(BlockPos, u32); 1]>,
    pub names: Vec<String>,
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
        let traits = self.kind.traits();
        traits.ticks && (!self.is_input() || traits.role == Some(Role::Input))
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
}
