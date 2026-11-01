//! The direct backend does not do code generation and interprets the netlist directly

mod compile;
mod node;
mod tick;
mod update;

use crate::backend::direct::node::ForwardLinks;
use crate::block_map::BlockMap;
use crate::compile_graph::SignalStrength;
use crate::engine::{Engine, EngineError, Input, NodeChanges, PendingTick};
use crate::netlist::{Netlist, NodeId};
use crate::worker::Worker;
use mchprs_blocks::blocks::ComparatorMode;
use mchprs_world::{TickEntry, TickPriority};
use node::{Node, NodeType, Nodes};
use std::mem;
use std::time::Instant;

pub fn start(netlist: &Netlist, ticks: &[TickEntry], io_only: bool) -> Worker {
    let block_map = BlockMap::new(netlist);
    let pending_ticks = block_map.map_ticks(ticks);
    let engine = DirectEngine::new(netlist, io_only, pending_ticks);
    Worker::start(block_map, Box::new(engine))
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

    fn set_power_and_propagate(&mut self, node_id: NodeId, power: SignalStrength) {
        let node = &mut self.nodes[node_id];
        let old_power = node.power;
        node.set_power(power);

        for forward_link in self.forward_links.get(&node.fwd_link_range) {
            let side = forward_link.side();
            let weight = forward_link.weight();
            let update = forward_link.node();

            let update_ref = &mut self.nodes[update];
            let inputs = if side {
                &mut update_ref.side_inputs
            } else {
                &mut update_ref.default_inputs
            };

            let old_input = old_power.saturating_sub(weight);
            let new_input = power.saturating_sub(weight);

            if old_input == new_input {
                continue;
            }

            inputs.update_power(old_input, new_input);

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
    fn run_ticks(&mut self, max_ticks: u64, deadline: Instant) -> Result<u64, EngineError> {
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
        Ok(completed)
    }

    fn input(&mut self, node_id: NodeId, input: Input) -> Result<(), EngineError> {
        self.changed_since_take = true;
        let node = self.node(node_id);
        match (input, &node.ty) {
            (Input::Interact, NodeType::Button) => {
                if node.is_powered() {
                    return Ok(());
                }
                self.scheduler
                    .schedule_tick(node_id, 10, TickPriority::Normal);
                self.set_power_and_propagate(node_id, SignalStrength::Fifteen);
            }
            (Input::Interact, NodeType::Lever) => {
                self.set_power_and_propagate(node_id, (!node.is_powered()).into());
            }
            (Input::PressurePlate { powered }, NodeType::PressurePlate) => {
                if node.is_powered() != powered {
                    self.set_power_and_propagate(node_id, powered.into());
                }
            }
            (input, ty) => {
                return Err(format!(
                    "node {node_id:?} is a {ty:?}, which does not accept {input:?}"
                )
                .into())
            }
        }
        Ok(())
    }

    fn take_changes(&mut self) -> Result<NodeChanges, EngineError> {
        let mut changes = NodeChanges::default();
        if !mem::take(&mut self.changed_since_take) {
            return Ok(changes);
        }
        for (index, node) in self.nodes.inner_mut().iter_mut().enumerate() {
            if node.changed && node.visible {
                node.changed = false;
                changes.states.push((NodeId::new(index), node.state()));
            }
        }
        changes.notes = mem::take(&mut self.notes);
        Ok(changes)
    }

    fn snapshot(&mut self) -> Result<(NodeChanges, Vec<PendingTick>), EngineError> {
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
        Ok((changes, self.scheduler.pending_ticks()))
    }
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

#[inline]
fn comparator_output_power(
    node: &Node,
    mode: ComparatorMode,
    far_input: Option<SignalStrength>,
) -> SignalStrength {
    let mut input_power = node.default_inputs.power();
    let side_power = node.side_inputs.power();
    if let Some(far_input) = far_input
        && input_power < SignalStrength::Fifteen
    {
        input_power = far_input;
    }
    let difference = input_power.get().wrapping_sub(side_power.get());
    if difference <= SignalStrength::Fifteen.get() {
        match mode {
            ComparatorMode::Compare => input_power,
            ComparatorMode::Subtract => SignalStrength::try_from(difference).unwrap(),
        }
    } else {
        SignalStrength::Zero
    }
}
