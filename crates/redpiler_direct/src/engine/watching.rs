use crate::node::{Nodes, Slot};
use mchprs_redpiler::circuit::NodeId;
use mchprs_simulation::watch::{Bit, Breakpoints, Hit, Watch};

const UNWATCHED: u32 = u32::MAX;

#[derive(Debug)]
struct WatchedNode {
    breakpoints: Vec<usize>,
}

#[derive(Debug)]
pub struct Watching {
    watched_by_slot: Box<[u32]>,
    nodes: Vec<WatchedNode>,
    breakpoints: Breakpoints<NodeId>,
    noticed: Vec<u32>,
    due: Vec<usize>,
}

impl Watching {
    /// Every watched node starts noticed, so the next tick checks what `previous` had not checked.
    pub fn new(
        watch: Watch<NodeId>,
        previous: Option<&Watching>,
        nodes: &Nodes,
    ) -> Option<Watching> {
        if watch.breakpoints.is_empty() {
            return None;
        }
        let mut watching = Watching {
            watched_by_slot: vec![UNWATCHED; nodes.inner().len()].into_boxed_slice(),
            nodes: Vec::new(),
            breakpoints: Breakpoints::default(),
            noticed: Vec::new(),
            due: Vec::new(),
        };
        let bit_nodes = watch.breakpoints.iter().flat_map(|breakpoint| {
            breakpoint.bits.iter().filter_map(|&bit| match bit {
                Bit::At(node) => Some(node),
                Bit::Fixed(_) => None,
            })
        });
        for node in bit_nodes {
            watching.add_node(nodes.get(node.index()));
        }
        for (index, breakpoint) in watch.breakpoints.iter().enumerate() {
            for &bit in &breakpoint.bits {
                if let Bit::At(node) = bit {
                    let watched = watching.watched_by_slot[node.index()] as usize;
                    watching.nodes[watched].breakpoints.push(index);
                }
            }
        }
        let previous = previous.map(|previous| &previous.breakpoints);
        watching.breakpoints = Breakpoints::new(watch.breakpoints, previous, powered(nodes));
        watching.noticed = (0..watching.nodes.len() as u32).collect();
        Some(watching)
    }

    fn add_node(&mut self, slot: Slot) {
        if self.watched_by_slot[slot.index()] != UNWATCHED {
            return;
        }
        self.watched_by_slot[slot.index()] = self.nodes.len() as u32;
        self.nodes.push(WatchedNode {
            breakpoints: Vec::new(),
        });
    }

    pub fn notice(&mut self, slot: Slot) {
        let index = self.watched_by_slot[slot.index()];
        if index != UNWATCHED {
            self.noticed.push(index);
        }
    }

    pub fn end_tick(&mut self, tick: u64, nodes: &Nodes) -> Option<Hit> {
        for index in self.noticed.drain(..) {
            let watched = &self.nodes[index as usize];
            self.due.extend_from_slice(&watched.breakpoints);
        }
        self.due.sort_unstable();
        self.due.dedup();
        let mut hit = None;
        for index in self.due.drain(..) {
            let fired = self.breakpoints.fire(index, tick, powered(nodes));
            hit = hit.or(fired);
        }
        hit
    }
}

fn powered(nodes: &Nodes) -> impl Fn(NodeId) -> bool + '_ {
    |node| nodes[nodes.get(node.index())].is_powered()
}
