use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::behavior::{InputStrengths, Pending};
use mchprs_redpiler::graph::{NodeKind, NodeState};
use std::fmt;
use std::num::NonZeroU128;
use std::ops::{Index, IndexMut, Range};

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct Slot(u32);

impl Slot {
    pub fn index(self) -> usize {
        self.0 as usize
    }
}

#[derive(Debug, Default)]
pub struct Nodes {
    nodes: Box<[Node]>,
}

impl Nodes {
    pub fn new(nodes: Box<[Node]>) -> Nodes {
        Nodes { nodes }
    }

    pub fn get(&self, idx: usize) -> Slot {
        if self.nodes.get(idx).is_some() {
            Slot(idx as u32)
        } else {
            panic!("node index out of bounds: {}", idx)
        }
    }

    pub fn inner(&self) -> &[Node] {
        &self.nodes
    }
}

impl Index<Slot> for Nodes {
    type Output = Node;

    fn index(&self, index: Slot) -> &Self::Output {
        // Safety: a slot comes from `get` on the one `Nodes` of its engine.
        unsafe { self.nodes.get_unchecked(index.0 as usize) }
    }
}

impl IndexMut<Slot> for Nodes {
    fn index_mut(&mut self, index: Slot) -> &mut Self::Output {
        unsafe { self.nodes.get_unchecked_mut(index.0 as usize) }
    }
}

#[derive(Clone, Copy)]
pub struct ForwardLink {
    data: u32,
}

impl ForwardLink {
    pub const MAX_NODES: usize = 1 << 27;

    pub fn new(id: Slot, side: bool, weight: u8) -> Self {
        assert!(id.index() < Self::MAX_NODES);
        Self {
            data: (id.index() as u32) << 5 | if side { 1 << 4 } else { 0 } | weight as u32,
        }
    }

    pub fn node(self) -> Slot {
        Slot(self.data >> 5)
    }

    pub fn side(self) -> bool {
        self.data & (1 << 4) != 0
    }

    pub fn weight(self) -> u8 {
        (self.data & 0b1111) as u8
    }
}

impl fmt::Debug for ForwardLink {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ForwardLink")
            .field("node", &self.node())
            .field("side", &self.side())
            .field("weight", &self.weight())
            .finish()
    }
}

#[derive(Clone, Debug, Default)]
pub struct ForwardLinkRange(Range<usize>);

#[derive(Debug, Default)]
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

    pub fn get(&self, range: &ForwardLinkRange) -> &[ForwardLink] {
        // Safety: a range comes from `extend` on the one `ForwardLinks` of its engine.
        unsafe { self.links.get_unchecked(range.0.clone()) }
    }
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
        // The counts add up to 255, so the input is zero if and only if the zero bucket holds 255.
        self.power_counts[0] != u8::MAX
    }

    pub fn power(&self) -> SignalStrength {
        let counts = u128::from_le_bytes(self.power_counts);
        // Safety: construction and updates preserve a total count of 255.
        let counts = unsafe { NonZeroU128::new_unchecked(counts) };
        SignalStrength::try_from(15 - (counts.leading_zeros() >> 3) as u8).unwrap()
    }
}

// Reading a node fetches a single cache line.
#[repr(align(64))]
#[derive(Debug, Clone)]
pub struct Node {
    pub kind: NodeKind,
    pub default_inputs: NodeInput,
    pub side_inputs: NodeInput,

    pub forward_links: ForwardLinkRange,

    pub state: NodeState,
    pub changed: bool,
    pub played_note: bool,
    pub due_tick: Option<u32>,
}

const _: () = assert!(size_of::<Node>() == 64);

impl Node {
    pub fn has_pending_tick(&self) -> bool {
        self.due_tick.is_some()
    }

    pub fn pending(&self, tick: u32) -> Pending {
        match self.due_tick {
            None => Pending::None,
            Some(due) if due == tick => Pending::DueThisTick,
            Some(_) => Pending::Later,
        }
    }

    pub fn inputs(&self) -> InputStrengths {
        InputStrengths {
            default: self.default_inputs.power(),
            side: self.side_inputs.power(),
        }
    }

    pub fn is_powered(&self) -> bool {
        !self.state.power.is_zero()
    }

    pub fn set_power(&mut self, power: SignalStrength) {
        self.state.power = power;
        self.changed = true;
    }

    pub fn set_repeater_locked(&mut self, locked: bool) {
        self.state.repeater_locked = locked;
        self.changed = true;
    }

    pub fn set_state(&mut self, state: NodeState) {
        self.state = state;
        self.changed = true;
    }
}
