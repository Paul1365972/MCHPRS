use std::{array, iter};

use mchprs_world::TickPriority;

use super::node::NodeId;

const SLOTS: usize = 16;
const PRIORITIES: [TickPriority; 4] = [
    TickPriority::Highest,
    TickPriority::Higher,
    TickPriority::High,
    TickPriority::Normal,
];
const LISTS: usize = SLOTS * PRIORITIES.len();
const NONE: u32 = u32::MAX;

// A node is pending in at most one list, so `next` holds one entry per node plus one sentinel
// head per list.
pub struct TickScheduler {
    next: Box<[u32]>,
    tails: [u32; LISTS],
    node_count: u32,
    slot: usize,
}

impl Default for TickScheduler {
    fn default() -> Self {
        Self::new(0)
    }
}

impl TickScheduler {
    pub fn new(node_count: usize) -> Self {
        let node_count = u32::try_from(node_count).unwrap();
        Self {
            next: vec![0; node_count as usize + LISTS].into_boxed_slice(),
            tails: array::from_fn(|list| node_count + list as u32),
            node_count,
            slot: 0,
        }
    }

    fn sentinel(&self, list: usize) -> u32 {
        self.node_count + list as u32
    }

    fn list(&self, delay: usize, priority: TickPriority) -> usize {
        (self.slot + delay) % SLOTS * PRIORITIES.len() + priority as usize
    }

    fn due(&self, list: usize) -> DueTicks {
        let tail = self.tails[list];
        if tail == self.sentinel(list) {
            DueTicks {
                node: NONE,
                last: NONE,
            }
        } else {
            DueTicks {
                node: self.next[self.sentinel(list) as usize],
                last: tail,
            }
        }
    }

    #[inline(always)]
    pub fn schedule(&mut self, node: NodeId, delay: usize, priority: TickPriority) {
        let list = self.list(delay, priority);
        let tail = self.tails[list] as usize;
        // Safety: tails hold node indices or sentinels, both inside `next`.
        unsafe { *self.next.get_unchecked_mut(tail) = node.index() as u32 };
        self.tails[list] = node.index() as u32;
    }

    pub fn advance(&mut self) {
        self.slot = (self.slot + 1) % SLOTS;
    }

    pub fn take_due(&mut self, priority: TickPriority) -> DueTicks {
        let list = self.list(0, priority);
        let due = self.due(list);
        self.tails[list] = self.sentinel(list);
        due
    }

    pub fn is_empty(&self) -> bool {
        (0..LISTS).all(|list| self.tails[list] == self.sentinel(list))
    }

    pub fn pending(&self) -> impl Iterator<Item = (NodeId, u32, TickPriority)> + '_ {
        (0..LISTS).flat_map(move |list| {
            let slot = list / PRIORITIES.len();
            let priority = PRIORITIES[list % PRIORITIES.len()];
            let delay = if slot > self.slot {
                slot - self.slot
            } else {
                slot + SLOTS - self.slot
            } as u32;
            let mut due = self.due(list);
            iter::from_fn(move || due.next_node(self).map(|node| (node, delay, priority)))
        })
    }
}

pub const fn priorities() -> [TickPriority; PRIORITIES.len()] {
    PRIORITIES
}

#[derive(Clone, Copy)]
pub struct DueTicks {
    node: u32,
    last: u32,
}

impl DueTicks {
    pub fn next_node(&mut self, scheduler: &TickScheduler) -> Option<NodeId> {
        if self.node == NONE {
            return None;
        }
        let node = self.node;
        // Read before the caller ticks the node, which may overwrite its entry.
        self.node = if node == self.last {
            NONE
        } else {
            scheduler.next[node as usize]
        };
        // Safety: lists only hold indices of the nodes the scheduler was sized for.
        Some(unsafe { NodeId::from_index(node as usize) })
    }
}
