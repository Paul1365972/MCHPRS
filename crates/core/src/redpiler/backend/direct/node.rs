use mchprs_blocks::blocks::ComparatorMode;
use smallvec::SmallVec;

use crate::redpiler::backend::common::{ForwardLink, NodeInput, NonMaxU8};
use crate::redpiler::backend::nodes::NodeStorage;

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
    NoteBlock {
        noteblock_id: u16,
    },
}

#[derive(Debug, Clone)]
pub struct Node {
    pub ty: NodeType,
    pub default_inputs: NodeInput,
    pub side_inputs: NodeInput,
    pub updates: SmallVec<[ForwardLink; 10]>,
    pub is_io: bool,

    /// Powered or lit
    pub powered: bool,
    /// Only for repeaters
    pub locked: bool,
    pub output_power: u8,
    pub changed: bool,
    pub pending_tick: bool,
}
