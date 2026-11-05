use mchprs_blocks::blocks::ComparatorMode;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::graph::NodeState;
use std::num::NonZeroU128;
use std::ops::{Index, IndexMut};

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct NodeId(u32);

impl NodeId {
    pub fn new(index: usize, node_count: usize) -> NodeId {
        assert!(index < node_count, "node index out of bounds: {index}");
        NodeId(index as u32)
    }

    pub fn index(self) -> usize {
        self.0 as usize
    }

    /// Safety: index must be within bounds of nodes array
    pub unsafe fn from_index(index: usize) -> NodeId {
        NodeId(index as u32)
    }
}

// This is Pretty Bad:tm: because one can create a NodeId using another instance of Nodes,
// but at least some type system protection is better than none.
#[derive(Default)]
pub struct Nodes {
    pub nodes: Box<[Node]>,
}

impl Nodes {
    pub fn new(nodes: Box<[Node]>) -> Nodes {
        Nodes { nodes }
    }

    pub fn get(&self, idx: usize) -> NodeId {
        if self.nodes.get(idx).is_some() {
            NodeId(idx as u32)
        } else {
            panic!("node index out of bounds: {}", idx)
        }
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

    // The index here MUST have been created by this instance, otherwise scary things will happen !
    fn index(&self, index: NodeId) -> &Self::Output {
        unsafe { self.nodes.get_unchecked(index.0 as usize) }
    }
}

impl IndexMut<NodeId> for Nodes {
    fn index_mut(&mut self, index: NodeId) -> &mut Self::Output {
        unsafe { self.nodes.get_unchecked_mut(index.0 as usize) }
    }
}

#[derive(Clone, Copy)]
pub struct ForwardLink {
    data: u32,
}

impl ForwardLink {
    pub fn new(id: NodeId, side: bool, weight: u8) -> Self {
        assert!(id.index() < (1 << 27));
        Self {
            data: (id.index() as u32) << 5 | if side { 1 << 4 } else { 0 } | weight as u32,
        }
    }

    pub fn node(self) -> NodeId {
        unsafe {
            // safety: ForwardLink is constructed using a NodeId
            NodeId::from_index((self.data >> 5) as usize)
        }
    }

    pub fn side(self) -> bool {
        self.data & (1 << 4) != 0
    }

    pub fn weight(self) -> u8 {
        (self.data & 0b1111) as u8
    }
}

impl std::fmt::Debug for ForwardLink {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ForwardLink")
            .field("node", &self.node())
            .field("side", &self.side())
            .field("weight", &self.weight())
            .finish()
    }
}

#[derive(Clone, Debug, Default)]
pub struct ForwardLinkRange(std::ops::Range<usize>);

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
#[repr(u8)]
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
    NoteBlock,
}

#[repr(align(16))]
#[derive(Debug, Clone)]
pub struct NodeInput {
    power_counts: [u8; 16],
}

impl NodeInput {
    pub const fn new() -> Self {
        let mut power_counts = [0; 16];
        power_counts[0] = u8::MAX;
        Self { power_counts }
    }

    #[inline]
    pub fn update_power(&mut self, old_power: SignalStrength, new_power: SignalStrength) {
        self.power_counts[old_power.get() as usize] -= 1;
        self.power_counts[new_power.get() as usize] += 1;
    }

    pub fn is_powered(&self) -> bool {
        // During compilation its ensured all signal strength buckets add up to 255
        // So if and only if the zero bucket contains 255 is the input zero
        self.power_counts[0] != u8::MAX
    }

    pub fn power(&self) -> SignalStrength {
        // Note: this might be slower on big-endian systems
        let counts = u128::from_le_bytes(self.power_counts);
        // Safety: construction and updates preserve a total count of 255.
        let counts = unsafe { NonZeroU128::new_unchecked(counts) };
        SignalStrength::try_from(15 - (counts.leading_zeros() >> 3) as u8).unwrap()
    }
}

// The `Node` struct's size is currently 64 bytes which happens to be the same
// size as an L1 cache line on most modern processors. By forcing a 64-byte
// alignment, we make sure that the entire `Node` can fit on one cache line,
// preventing scenarios where we have to fetch 2 cache lines to read a single `Node`.
#[repr(align(64))]
#[derive(Debug, Clone)]
pub struct Node {
    pub kind: NodeKind,
    pub default_inputs: NodeInput,
    pub side_inputs: NodeInput,

    pub fwd_link_range: ForwardLinkRange,

    pub visible: bool,

    pub power: SignalStrength,
    /// Only for repeaters
    pub repeater_locked: bool,
    pub changed: bool,
    pub due_tick: Option<u32>,
}

impl Node {
    pub fn has_pending_tick(&self) -> bool {
        self.due_tick.is_some()
    }

    /// Like vanilla's `hasScheduledTick`, which no longer counts a tick collected to run in `tick`
    pub fn has_scheduled_tick(&self, tick: u32) -> bool {
        self.due_tick.is_some_and(|due| due != tick)
    }

    pub fn is_powered(&self) -> bool {
        !self.power.is_zero()
    }

    pub fn state(&self) -> NodeState {
        NodeState {
            power: self.power,
            repeater_locked: self.repeater_locked,
        }
    }

    pub fn set_power(&mut self, power: SignalStrength) {
        self.power = power;
        self.changed = true;
    }

    pub fn set_powered(&mut self, powered: bool) {
        self.set_power(powered.into());
    }

    pub fn set_repeater_locked(&mut self, locked: bool) {
        self.repeater_locked = locked;
        self.changed = true;
    }
}
