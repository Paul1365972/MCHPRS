use mchprs_blocks::blocks::{ComparatorMode, Instrument};
use mchprs_blocks::BlockPos;
use petgraph::stable_graph::{NodeIndex, StableGraph};

pub type NodeIdx = NodeIndex;

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum NodeType {
    Repeater {
        delay: u8,
        facing_diode: bool,
    },
    Torch,
    Comparator {
        mode: ComparatorMode,
        far_input: Option<u8>,
        facing_diode: bool,
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
        note: u32,
    },
    ComparatorLine {
        states: Box<[u8]>,
    },
    ExternalInput,
    ExternalOutput {
        target_idx: NodeIdx,
        delay: u32,
    },
}

#[derive(Debug, Clone, Default)]
pub struct NodeState {
    pub powered: bool,
    pub repeater_locked: bool,
    pub output_strength: u8,
}

impl NodeState {
    pub fn simple(powered: bool) -> NodeState {
        NodeState {
            powered,
            output_strength: if powered { 15 } else { 0 },
            ..Default::default()
        }
    }

    pub fn repeater(powered: bool, locked: bool) -> NodeState {
        NodeState {
            powered,
            repeater_locked: locked,
            output_strength: if powered { 15 } else { 0 },
        }
    }

    pub fn ss(ss: u8) -> NodeState {
        NodeState {
            output_strength: ss,
            ..Default::default()
        }
    }

    pub fn comparator(powered: bool, ss: u8) -> NodeState {
        NodeState {
            powered,
            output_strength: ss,
            ..Default::default()
        }
    }
}

#[derive(Debug, Default)]
pub struct Annotations {
    pub separate: Option<u32>,
}

#[derive(Debug)]
pub struct CompileNode {
    pub ty: NodeType,
    pub block: Option<(BlockPos, u32)>,
    pub state: NodeState,

    pub is_input: bool,
    pub is_output: bool,
    pub annotations: Annotations,
}

impl CompileNode {
    pub fn is_removable(&self) -> bool {
        !self.is_input && !self.is_output && self.annotations.separate.is_none()
    }

    pub fn is_io_and_flushable(&self) -> bool {
        (self.is_input || self.is_output) && self.block.is_some()
    }

    pub fn can_be_ticked(&self) -> bool {
        matches!(
            self.ty,
            NodeType::Repeater { .. }
                | NodeType::Comparator { .. }
                | NodeType::Torch
                | NodeType::Button
                | NodeType::Lamp
                | NodeType::ComparatorLine { .. }
        )
    }

    pub fn is_analog(&self) -> bool {
        matches!(
            self.ty,
            NodeType::Comparator { .. }
                | NodeType::Wire
                | NodeType::ExternalOutput { .. }
                | NodeType::ComparatorLine { .. }
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkType {
    Default,
    Side,
}

#[derive(Debug)]
pub struct CompileLink {
    pub ty: LinkType,
    pub ss: u8,
}

impl CompileLink {
    pub fn new(ty: LinkType, ss: u8) -> CompileLink {
        CompileLink { ty, ss }
    }

    pub fn default(ss: u8) -> CompileLink {
        CompileLink {
            ty: LinkType::Default,
            ss,
        }
    }

    pub fn side(ss: u8) -> CompileLink {
        CompileLink {
            ty: LinkType::Side,
            ss,
        }
    }
}

pub type CompileGraph = StableGraph<CompileNode, CompileLink>;
