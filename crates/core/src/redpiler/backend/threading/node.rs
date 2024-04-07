use mchprs_blocks::blocks::ComparatorMode;
use smallvec::SmallVec;

use crate::redpiler::backend::common::{ForwardLink, NodeInput, NonMaxU8};
use crate::redpiler::backend::nodes::{NodeId, NodeStorage};

pub type Nodes = NodeStorage<Node>;

#[derive(Debug, Clone, Copy)]
pub enum NodeType {
    Repeater {
        delay: u8,
        facing_diode: bool,
    },
    Torch,
    Comparator {
        mode: ComparatorMode,
        far_input: Option<NonMaxU8>,
        facing_diode: bool,
    },
    Lamp,
    Button,
    Lever,
    PressurePlate,
    Trapdoor,
    Wire,
    Constant,
    ExternalInput,
    ExternalOutput {
        output_index: u8,
        target_id: NodeId,
    },
    ComparatorLine {
        delay: u8,
    },
}

// struct is 128 bytes to fit nicely into cachelines
// which are usualy 64 bytes, it can vary but is almost always a power of 2
// Note: Removing the alignment and reducing the updates smallvec to 8 seems to have no performance impact
#[derive(Debug, Clone)]
#[repr(align(128))]
pub struct Node {
    pub ty: NodeType,
    pub default_inputs: NodeInput,
    pub side_inputs: NodeInput,
    pub default_power: u8,
    pub side_power: u8,
    pub updates: SmallVec<[ForwardLink; 18]>,
    pub is_io: bool,

    /// Powered or lit
    pub powered: bool,
    pub output_power: u8,
    pub changed: bool,
    pub pending_tick: bool,
}

impl Node {
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
