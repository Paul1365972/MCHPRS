//! The direct backend does not do code generation and interprets the netlist directly

mod compile;
mod node;
mod tick;
mod update;

use crate::backend::direct::node::ForwardLinks;
use crate::backend::Backend;
use crate::engine::{Engine, Input, NodeChanges, PendingTick};
use crate::netlist::{Netlist, NodeId};
use crate::worker::Worker;
use mchprs_blocks::blocks::ComparatorMode;
use mchprs_redstone::bool_to_ss;
use mchprs_world::{TickEntry, TickPriority};
use node::{Node, NodeType, Nodes};
use std::mem;
use std::time::Instant;

pub fn start(netlist: Netlist, ticks: Vec<TickEntry>, io_only: bool) -> impl Backend {
    Worker::start(netlist, ticks, move |netlist, pending_ticks| {
        DirectEngine::new(&netlist, io_only, pending_ticks)
    })
}

#[derive(Default, Clone)]
struct Queues([Vec<NodeId>; TickScheduler::NUM_PRIORITIES]);

impl Queues {
    #[inline(always)]
    fn drain_each<F: FnMut(NodeId)>(&mut self, mut f: F) {
        for q in self.0.iter_mut() {
            for n in q.iter() {
                f(*n);
            }
            q.clear();
        }
    }
}

#[derive(Default)]
struct TickScheduler {
    queues_deque: [Queues; Self::NUM_QUEUES],
    pos: usize,
}

impl TickScheduler {
    const NUM_PRIORITIES: usize = 4;
    const NUM_QUEUES: usize = 16;

    fn pending_ticks(&self) -> Vec<PendingTick> {
        let mut pending = Vec::new();
        for (idx, queues) in self.queues_deque.iter().enumerate() {
            let delay = if self.pos >= idx {
                idx + Self::NUM_QUEUES
            } else {
                idx
            } - self.pos;
            for (entries, priority) in queues.0.iter().zip(Self::priorities()) {
                for &node in entries {
                    pending.push(PendingTick {
                        node,
                        delay: delay as u32,
                        priority,
                    });
                }
            }
        }
        pending
    }

    fn schedule_tick(&mut self, node: NodeId, delay: usize, priority: TickPriority) {
        self.queues_deque[(self.pos + delay) % Self::NUM_QUEUES].0[priority as usize].push(node);
    }

    fn queues_this_tick(&mut self) -> Queues {
        self.pos = (self.pos + 1) % Self::NUM_QUEUES;
        mem::take(&mut self.queues_deque[self.pos])
    }

    fn end_tick(&mut self, queues: Queues) {
        self.queues_deque[self.pos % Self::NUM_QUEUES] = queues;
    }

    fn priorities() -> [TickPriority; Self::NUM_PRIORITIES] {
        [
            TickPriority::Highest,
            TickPriority::Higher,
            TickPriority::High,
            TickPriority::Normal,
        ]
    }
}

pub struct DirectEngine {
    nodes: Nodes,
    forward_links: ForwardLinks,
    scheduler: TickScheduler,
    notes: Vec<NodeId>,
    work: usize,
    changed_since_take: bool,
}

impl DirectEngine {
    const WORK_PER_TIME_CHECK: usize = 20_000;

    pub fn new(netlist: &Netlist, io_only: bool, pending_ticks: Vec<PendingTick>) -> DirectEngine {
        compile::compile(netlist, io_only, pending_ticks)
    }

    fn node(&self, node_id: NodeId) -> &Node {
        &self.nodes.inner()[node_id.index()]
    }

    fn tick(&mut self) {
        let mut queues = self.scheduler.queues_this_tick();
        let mut work = 1;
        queues.drain_each(|node_id| {
            work += 1;
            self.tick_node(node_id);
        });
        self.work += work;
        self.scheduler.end_tick(queues);
    }

    fn schedule_tick(&mut self, node_id: NodeId, delay: usize, priority: TickPriority) {
        self.scheduler.schedule_tick(node_id, delay, priority);
    }

    fn set_node(&mut self, node_id: NodeId, powered: bool, new_power: u8) {
        let node = &mut self.nodes[node_id];
        let old_power = node.output_power;

        node.changed = true;
        node.powered = powered;
        node.output_power = new_power;

        for forward_link in self.forward_links.get(&node.fwd_link_range) {
            let side = forward_link.side();
            let distance = forward_link.ss();
            let update = forward_link.node();

            let update_ref = &mut self.nodes[update];
            let inputs = if side {
                &mut update_ref.side_inputs
            } else {
                &mut update_ref.default_inputs
            };

            let old_power = old_power.saturating_sub(distance);
            let new_power = new_power.saturating_sub(distance);

            if old_power == new_power {
                continue;
            }

            // Safety: signal strength is never larger than 15
            unsafe {
                *inputs.ss_counts.get_unchecked_mut(old_power as usize) -= 1;
                *inputs.ss_counts.get_unchecked_mut(new_power as usize) += 1;
            }

            update::update_node(
                &mut self.scheduler,
                &mut self.notes,
                &mut self.nodes,
                update,
            );
        }
    }
}

impl Engine for DirectEngine {
    fn run_ticks(&mut self, max_ticks: u64, deadline: Instant) -> u64 {
        let mut remaining = max_ticks;
        self.work = 0;
        while remaining != 0 {
            self.tick();
            remaining -= 1;
            if self.work >= Self::WORK_PER_TIME_CHECK {
                self.work = 0;
                if Instant::now() >= deadline {
                    break;
                }
            }
        }
        let completed = max_ticks - remaining;
        self.changed_since_take |= completed != 0;
        completed
    }

    fn input(&mut self, node_id: NodeId, input: Input) {
        self.changed_since_take = true;
        let node = self.node(node_id);
        match (input, &node.ty) {
            (Input::Interact, NodeType::Button) => {
                if node.powered {
                    return;
                }
                self.schedule_tick(node_id, 10, TickPriority::Normal);
                self.set_node(node_id, true, 15);
            }
            (Input::Interact, NodeType::Lever) => {
                self.set_node(node_id, !node.powered, bool_to_ss(!node.powered));
            }
            (Input::PressurePlate { powered }, NodeType::PressurePlate) => {
                if node.powered != powered {
                    self.set_node(node_id, powered, bool_to_ss(powered));
                }
            }
            _ => unreachable!(
                "node {node_id:?} is a {:?}, which does not accept {input:?}",
                node.ty
            ),
        }
    }

    fn take_changes(&mut self) -> NodeChanges {
        let mut changes = NodeChanges::default();
        if !mem::take(&mut self.changed_since_take) {
            return changes;
        }
        for (index, node) in self.nodes.inner_mut().iter_mut().enumerate() {
            if node.changed && node.visible {
                node.changed = false;
                changes.states.push((NodeId::new(index), node.state()));
            }
        }
        changes.notes = mem::take(&mut self.notes);
        changes
    }

    fn snapshot(&mut self) -> (NodeChanges, Vec<PendingTick>) {
        self.changed_since_take = false;
        let states = self
            .nodes
            .inner_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(index, node)| {
                mem::take(&mut node.changed).then(|| (NodeId::new(index), node.state()))
            })
            .collect();
        let changes = NodeChanges {
            states,
            notes: mem::take(&mut self.notes),
        };
        (changes, self.scheduler.pending_ticks())
    }
}

/// Set node for use in `update`. None of the nodes here have usable output power,
/// so this function does not set that.
fn set_node(node: &mut Node, powered: bool) {
    node.powered = powered;
    node.changed = true;
}

fn set_node_locked(node: &mut Node, locked: bool) {
    node.locked = locked;
    node.changed = true;
}

fn schedule_tick(
    scheduler: &mut TickScheduler,
    node_id: NodeId,
    node: &mut Node,
    delay: usize,
    priority: TickPriority,
) {
    node.pending_tick = true;
    scheduler.schedule_tick(node_id, delay, priority);
}

fn get_bool_input(node: &Node) -> bool {
    // During compilation its ensured all signal strength buckets add up to 255
    // So if and only if the zero bucket contains 255 is the input zero
    node.default_inputs.ss_counts[0] != 255
}

fn get_bool_side(node: &Node) -> bool {
    node.side_inputs.ss_counts[0] != 255
}

fn last_index_positive(array: &[u8; 16]) -> u32 {
    // Note: this might be slower on big-endian systems
    let value = u128::from_le_bytes(*array);
    if value == 0 {
        0
    } else {
        15 - (value.leading_zeros() >> 3)
    }
}

fn get_all_input(node: &Node) -> (u8, u8) {
    let input_power = last_index_positive(&node.default_inputs.ss_counts) as u8;

    let side_input_power = last_index_positive(&node.side_inputs.ss_counts) as u8;

    (input_power, side_input_power)
}

// This function is optimized for input values from 0 to 15 and does not work correctly outside that
// range
fn calculate_comparator_output(mode: ComparatorMode, input_strength: u8, power_on_sides: u8) -> u8 {
    let difference = input_strength.wrapping_sub(power_on_sides);
    if difference <= 15 {
        match mode {
            ComparatorMode::Compare => input_strength,
            ComparatorMode::Subtract => difference,
        }
    } else {
        0
    }
}
