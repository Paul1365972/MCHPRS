use crate::compile_graph::NodeState;
use crate::netlist::NodeId;
use mchprs_blocks::blocks::ComparatorMode;
use std::num::NonZeroU8;
use std::ops::{Index, IndexMut};

// Only NodeIds from this backend's own forward links and tick queues are indexed
// unchecked; the trait entry points index checked.
pub struct Nodes {
    nodes: Box<[Node]>,
}

impl Nodes {
    pub fn new(nodes: Box<[Node]>) -> Nodes {
        Nodes { nodes }
    }

    pub fn inner(&self) -> &[Node] {
        &self.nodes
    }

    pub fn inner_mut(&mut self) -> &mut [Node] {
        &mut self.nodes
    }
}

impl Index<NodeId> for Nodes {
    type Output = Node;

    fn index(&self, index: NodeId) -> &Self::Output {
        unsafe { self.nodes.get_unchecked(index.index()) }
    }
}

impl IndexMut<NodeId> for Nodes {
    fn index_mut(&mut self, index: NodeId) -> &mut Self::Output {
        unsafe { self.nodes.get_unchecked_mut(index.index()) }
    }
}

#[derive(Clone, Copy)]
pub struct ForwardLink {
    data: u32,
}

impl ForwardLink {
    pub fn new(id: NodeId, side: bool, ss: u8) -> Self {
        assert!(id.index() < (1 << 27));
        // the clamp_weights compile pass should ensure ss < 15
        assert!(ss < 15);
        Self {
            data: (id.index() as u32) << 5 | if side { 1 << 4 } else { 0 } | ss as u32,
        }
    }

    pub fn node(self) -> NodeId {
        NodeId::from_raw(self.data >> 5)
    }

    pub fn side(self) -> bool {
        self.data & (1 << 4) != 0
    }

    pub fn ss(self) -> u8 {
        (self.data & 0b1111) as u8
    }
}

impl std::fmt::Debug for ForwardLink {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ForwardLink")
            .field("node", &self.node())
            .field("side", &self.side())
            .field("ss", &self.ss())
            .finish()
    }
}

#[derive(Clone, Debug, Default)]
pub struct ForwardLinkRange(std::ops::Range<usize>);

impl ForwardLinkRange {
    pub fn len(&self) -> usize {
        self.0.end - self.0.start
    }
}

#[derive(Default)]
pub struct ForwardLinks {
    links: Vec<ForwardLink>,
}

impl ForwardLinks {
    pub fn extend(&mut self, iter: impl IntoIterator<Item = ForwardLink>) -> ForwardLinkRange {
        let start = self.links.len();
        self.links.extend(iter);
        let end = self.links.len();

        ForwardLinkRange(start..end)
    }

    /// The `range` MUST have been created by this instance of ForwardLinks, otherwise this is UB.
    pub fn get(&self, range: &ForwardLinkRange) -> &[ForwardLink] {
        // Safety: there's only one instance of ForwardLinks in the backend
        unsafe { self.links.get_unchecked(range.0.clone()) }
    }
}

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
    NoteBlock,
}

#[repr(align(16))]
#[derive(Debug, Clone, Default)]
pub struct NodeInput {
    pub ss_counts: [u8; 16],
}

#[derive(Debug, Clone, Copy)]
pub struct NonMaxU8(NonZeroU8);

impl NonMaxU8 {
    pub fn new(value: u8) -> Option<Self> {
        NonZeroU8::new(value + 1).map(Self)
    }

    pub fn get(self) -> u8 {
        self.0.get() - 1
    }
}

// The `Node` struct's size is currently 64 bytes which happens to be the same
// size as an L1 cache line on most modern processors. By forcing a 64-byte
// alignment, we make sure that the entire `Node` can fit on one cache line,
// preventing scenarios where we have to fetch 2 cache lines to read a single `Node`.
#[repr(align(64))]
#[derive(Debug, Clone)]
pub struct Node {
    pub ty: NodeType,
    pub default_inputs: NodeInput,
    pub side_inputs: NodeInput,

    pub fwd_link_range: ForwardLinkRange,

    pub visible: bool,

    /// Powered or lit
    pub powered: bool,
    /// Only for repeaters
    pub locked: bool,
    pub output_power: u8,
    pub changed: bool,
    pub pending_tick: bool,
}

impl Node {
    pub fn state(&self) -> NodeState {
        NodeState {
            powered: self.powered,
            repeater_locked: self.locked,
            output_strength: self.output_power,
        }
    }
}
