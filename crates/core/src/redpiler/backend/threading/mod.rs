mod compile;
mod node;
mod tick;
mod update;

use super::common::{calculate_comparator_output, NodeIdWithData, NonMaxU8};
use super::nodes::NodeId;
use super::JITBackend;
use crate::redpiler::compile_graph::CompileGraph;
use crate::redpiler::{block_powered_mut, CompilerOptions, TaskMonitor};
use crate::redstone::bool_to_ss;
use crate::world::World;
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::BlockPos;
use mchprs_world::{TickEntry, TickPriority};
use node::Node;
use node::NodeType;
use node::Nodes;
use petgraph::visit::EdgeRef;
use realtime_channel::{Receiver, Sender};
use rustc_hash::FxHashMap;
use std::collections::BinaryHeap;
use std::fmt::Debug;
use std::sync::Arc;
use std::thread::JoinHandle;
use std::{array, fmt, mem};
use tracing::{debug, warn};

const WORK_CHANNEL_SIZE: usize = 1 << 10;
const UPDATES_CHANNEL_SIZE: usize = 1 << 20;
const CHANGES_CHANNEL_SIZE: usize = 1 << 20;
const FLUSH_OFFSET: usize = 2;

#[derive(Clone)]
struct Queues([Vec<NodeIdWithData>; TickScheduler::NUM_PRIORITIES]);

impl Queues {
    unsafe fn new(capacity: usize) -> Self {
        Self(array::from_fn(|_| Vec::with_capacity(capacity)))
    }
}

struct TickScheduler {
    queues_deque: [Queues; Self::NUM_QUEUES],
    pos: usize,
}

impl TickScheduler {
    const NUM_PRIORITIES: usize = 4;
    const NUM_QUEUES: usize = 64;

    unsafe fn new(capacity: usize) -> Self {
        Self {
            queues_deque: std::array::from_fn(|_| Queues::new(capacity)),
            pos: 0,
        }
    }

    fn schedule_tick(&mut self, node_data: NodeIdWithData, delay: usize, priority: TickPriority) {
        debug_assert!(delay < Self::NUM_QUEUES);
        let queues_index = (self.pos + delay) % Self::NUM_QUEUES;
        let queue = &mut self.queues_deque[queues_index].0[priority as usize];
        debug_assert!(queue.len() < queue.capacity());
        unsafe {
            queue.as_mut_ptr().add(queue.len()).write(node_data);
            queue.set_len(queue.len() + 1);
        }
    }

    fn queues_this_tick(&mut self) -> Queues {
        self.pos = (self.pos + 1) % Self::NUM_QUEUES;
        let dummy = Queues([vec![], vec![], vec![], vec![]]);
        mem::replace(&mut self.queues_deque[self.pos], dummy)
    }

    fn end_tick(&mut self, queues: Queues) {
        let dummy = mem::replace(&mut self.queues_deque[self.pos % Self::NUM_QUEUES], queues);
        mem::forget(dummy);
    }

    fn reset<W: World>(&mut self, world: &mut W, blocks: &[Option<(BlockPos, Block)>]) {
        for (idx, queues) in self.queues_deque.iter().enumerate() {
            let delay = if self.pos >= idx {
                idx + Self::NUM_QUEUES
            } else {
                idx
            } - self.pos;
            for (entries, priority) in queues.0.iter().zip(Self::priorities()) {
                for node in entries {
                    let Some((pos, _)) = blocks[node.node().index()] else {
                        warn!("Cannot schedule tick for node {:?} because block information is missing", node);
                        continue;
                    };
                    world.schedule_tick(pos, delay as u32, priority);
                }
            }
        }
        for queues in self.queues_deque.iter_mut() {
            for queue in queues.0.iter_mut() {
                queue.clear();
            }
        }
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

#[derive(Default)]
pub struct ThreadingBackend {
    handles: Vec<JoinHandle<Worker>>,
    change_receivers: Vec<Receiver<ChangeMessage>>,
    work_senders: Vec<Sender<WorkerMessage>>,
    pos_map: FxHashMap<BlockPos, (WorkerId, NodeId)>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct WorkerId(u16);

struct Worker {
    id: WorkerId,
    inputs: Vec<Receiver<UpdateMessage>>,
    outputs: Vec<Sender<UpdateMessage>>,
    work_receiver: Receiver<WorkerMessage>,
    change_sender: Sender<ChangeMessage>,
    nodes: Nodes,
    scheduler: TickScheduler,
    blocks: Vec<Option<(BlockPos, Block)>>,
    last_io_index: usize,
}

enum UpdateMessage {
    Update {
        node_id: NodeId,
        signal_strength: u8,
    },
    UpdatesEnd,
}

enum WorkerMessage {
    Tick,
    UseBlock { node_id: NodeId },
    PressurePlate { node_id: NodeId, powered: bool },
    Inspect { node_id: NodeId },
    Flush { io_only: bool },
    Reset,
}

enum ChangeMessage {
    Change { pos: BlockPos, block: Block },
    ChangesEnd,
}

impl Worker {
    fn set_node(&mut self, node_id: NodeId, powered: bool, new_power: u8) {
        let node = &mut self.nodes[node_id];
        let old_power = node.output_power;

        node.changed = true;
        node.powered = powered;
        node.output_power = new_power;
        for i in 0..node.updates.len() {
            let node = &self.nodes[node_id];
            let update_link = unsafe { *node.updates.get_unchecked(i) };
            let side = update_link.side();
            let distance = update_link.ss();
            let update = update_link.node();

            let update_ref = &mut self.nodes[update];
            if update_ref.is_analog() {
                let old_power = old_power.saturating_sub(distance);
                let new_power = new_power.saturating_sub(distance);
                if old_power == new_power {
                    continue;
                }

                let (input, inputs) = if side {
                    (&mut update_ref.side_power, &mut update_ref.side_inputs)
                } else {
                    (
                        &mut update_ref.default_power,
                        &mut update_ref.default_inputs,
                    )
                };

                unsafe {
                    *inputs.ss_counts.get_unchecked_mut(old_power as usize) -= 1;
                    *inputs.ss_counts.get_unchecked_mut(new_power as usize) += 1;
                }

                let power = inputs.get_ss();

                if *input == power {
                    continue;
                }
                *input = power;

                if side {
                    update::update_node_side_analog(&mut self.scheduler, update, update_ref);
                } else {
                    update::update_node_default_analog(
                        &mut self.scheduler,
                        &mut self.outputs,
                        update,
                        update_ref,
                    );
                }
            } else {
                let old_power = old_power > distance;
                let new_power = new_power > distance;

                if old_power == new_power {
                    continue;
                }

                let input = if side {
                    &mut update_ref.side_power
                } else {
                    &mut update_ref.default_power
                };

                if new_power {
                    *input += 1;
                    if *input != 1 {
                        continue;
                    }
                } else {
                    *input -= 1;
                    if *input != 0 {
                        continue;
                    }
                }

                if side {
                    update::update_node_side_digital(&mut self.scheduler, update, update_ref);
                } else {
                    update::update_node_default_digital(&mut self.scheduler, update, update_ref);
                }
            }
            //update_node(&mut self.scheduler, &mut self.outputs, update, update_ref);
        }
    }
}

impl JITBackend for ThreadingBackend {
    fn inspect(&mut self, pos: BlockPos) {
        let Some(&(worker_id, node_id)) = self.pos_map.get(&pos) else {
            debug!("could not find node at pos {}", pos);
            return;
        };
        self.work_senders[worker_id.0 as usize].send(WorkerMessage::Inspect { node_id });
    }

    fn reset<W: World>(&mut self, world: &mut W, io_only: bool) {
        for _ in 0..FLUSH_OFFSET {
            for recv in self.change_receivers.iter_mut() {
                while let ChangeMessage::Change { pos, block } = recv.recv() {
                    world.set_block(pos, block);
                }
            }
        }

        for worker in self.work_senders.iter_mut() {
            worker.send(WorkerMessage::Reset);
        }
        for handle in self.handles.drain(..) {
            let mut worker = handle.join().unwrap();

            worker.scheduler.reset(world, &worker.blocks);

            let nodes = std::mem::take(&mut worker.nodes);

            for (i, node) in nodes.into_inner().iter().enumerate() {
                let Some((pos, block)) = worker.blocks[i] else {
                    continue;
                };
                if matches!(node.ty, NodeType::Comparator { .. }) {
                    let block_entity = BlockEntity::Comparator {
                        output_strength: node.output_power,
                    };
                    world.set_block_entity(pos, block_entity);
                }

                if io_only && !node.is_io {
                    world.set_block(pos, block);
                }
            }
        }
        self.handles.clear();
        self.change_receivers.clear();
        self.work_senders.clear();
        self.pos_map.clear();
    }

    fn on_use_block(&mut self, pos: BlockPos) {
        let (worker_id, node_id) = self.pos_map[&pos];
        self.work_senders[worker_id.0 as usize].send(WorkerMessage::UseBlock { node_id });
    }

    fn set_pressure_plate(&mut self, pos: BlockPos, powered: bool) {
        let (worker_id, node_id) = self.pos_map[&pos];
        self.work_senders[worker_id.0 as usize]
            .send(WorkerMessage::PressurePlate { node_id, powered });
    }

    fn tick(&mut self) {
        for worker in self.work_senders.iter_mut() {
            worker.send(WorkerMessage::Tick);
        }
    }

    fn compile(
        &mut self,
        graph: CompileGraph,
        ticks: Vec<TickEntry>,
        options: &CompilerOptions,
        monitor: Arc<TaskMonitor>,
    ) {
        compile::compile(self, graph, ticks, options, monitor);
    }

    fn flush<W: World>(&mut self, world: &mut W, io_only: bool) {
        for root in self.work_senders.iter_mut() {
            root.send(WorkerMessage::Flush { io_only });
        }
        for recv in self.change_receivers.iter_mut() {
            while let ChangeMessage::Change { pos, block } = recv.recv() {
                world.set_block(pos, block);
            }
        }
    }
}

impl Worker {
    fn handle_tick(&mut self) {
        let mut queues = self.scheduler.queues_this_tick();

        for sender in self.outputs.iter_mut() {
            sender.begin_commit(1024);
        }

        for queue in queues.0.iter_mut() {
            let mut inputs = std::mem::take(&mut self.inputs);
            for input in inputs.iter_mut() {
                while let UpdateMessage::Update {
                    node_id,
                    signal_strength,
                } = input.recv()
                {
                    self.set_node(node_id, signal_strength > 0, signal_strength);
                }
            }
            self.inputs = inputs;

            for node_data in queue.drain(..) {
                self.tick_node(node_data);
            }

            for output in self.outputs.iter_mut() {
                output.send_unsafe(UpdateMessage::UpdatesEnd);
            }
        }

        for sender in self.outputs.iter_mut() {
            sender.end_commit();
        }

        self.scheduler.end_tick(queues);
    }

    fn handle_use_block(&mut self, node_id: NodeId) {
        let node = &self.nodes[node_id];
        match node.ty {
            NodeType::Button => {
                if !node.powered {
                    self.scheduler.schedule_tick(
                        NodeIdWithData::new(node_id),
                        10,
                        TickPriority::Normal,
                    );
                    self.set_node(node_id, true, 15);
                }
            }
            NodeType::Lever => {
                self.set_node(node_id, !node.powered, bool_to_ss(!node.powered));
            }
            _ => warn!("Tried to use a {:?} redpiler node", node.ty),
        }
    }

    fn handle_pressure_plate(&mut self, node_id: NodeId, powered: bool) {
        let node = &self.nodes[node_id];
        match node.ty {
            NodeType::PressurePlate => {
                self.set_node(node_id, powered, bool_to_ss(powered));
            }
            _ => warn!("Tried to set pressure plate state for a {:?}", node.ty),
        }
    }

    fn handle_flush(&mut self, io_only: bool) {
        let num = if io_only {
            self.last_io_index + 1
        } else {
            self.nodes.len()
        };
        self.change_sender.begin_commit(num + 1);

        for i in 0..num {
            let Some((pos, block)) = &mut self.blocks[i] else {
                continue;
            };
            let idx = unsafe { NodeId::from_index(i) };
            let node = &mut self.nodes[idx];
            if node.changed {
                if let Some(powered) = block_powered_mut(block) {
                    *powered = node.powered
                }
                if let Block::RedstoneWire { wire, .. } = block {
                    wire.power = node.output_power
                };
                if let Block::RedstoneRepeater { repeater } = block {
                    repeater.locked = node.side_power > 0;
                }
                self.change_sender.send_unsafe(ChangeMessage::Change {
                    pos: *pos,
                    block: *block,
                });
                node.changed = false;
            }
        }
        self.change_sender.send_unsafe(ChangeMessage::ChangesEnd);
        self.change_sender.end_commit();
    }

    fn run(mut self) -> Self {
        loop {
            match self.work_receiver.recv() {
                WorkerMessage::Tick => self.handle_tick(),
                WorkerMessage::UseBlock { node_id } => self.handle_use_block(node_id),
                WorkerMessage::PressurePlate { node_id, powered } => {
                    self.handle_pressure_plate(node_id, powered)
                }
                WorkerMessage::Inspect { node_id } => {
                    debug!("Node {:?}: {:#?}", node_id, self.nodes[node_id])
                }
                WorkerMessage::Flush { io_only } => self.handle_flush(io_only),
                WorkerMessage::Reset => return self,
            }
        }
    }
}

/// Set node for use in `update`. None of the nodes here have usable output power,
/// so this function does not set that.
fn set_node(node: &mut Node, powered: bool) {
    node.powered = powered;
    node.changed = true;
}

fn schedule_tick(
    scheduler: &mut TickScheduler,
    node_data: NodeIdWithData,
    node: &mut Node,
    delay: usize,
    priority: TickPriority,
) {
    node.pending_tick = true;
    scheduler.schedule_tick(node_data, delay, priority);
}

fn get_bool_input(node: &Node) -> bool {
    node.default_inputs.get_bool()
}

fn get_bool_side(node: &Node) -> bool {
    node.default_inputs.get_bool()
}

fn get_analog_input(node: &Node) -> u8 {
    node.default_inputs.get_ss()
}

fn get_analog_side(node: &Node) -> u8 {
    node.side_inputs.get_ss()
}

impl fmt::Display for Worker {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("digraph{")?;
        for (id, node) in self.nodes.inner().iter().enumerate() {
            if matches!(node.ty, NodeType::Wire) {
                continue;
            }
            let label = match node.ty {
                NodeType::Constant => format!("Constant: {}", node.output_power),
                _ => format!("{:?}", node.ty)
                    .split_whitespace()
                    .next()
                    .unwrap()
                    .to_string(),
            };
            let pos = if let Some((pos, _)) = self.blocks[id] {
                format!("{}, {}, {}", pos.x, pos.y, pos.z)
            } else {
                "No Pos".to_string()
            };
            write!(f, "n{}[label=\"{}\\n({})\"];", id, label, pos,)?;
            for link in node.updates.iter() {
                let out_index = link.node().index();
                let distance = link.ss();
                let color = if link.side() { ",color=\"blue\"" } else { "" };
                write!(
                    f,
                    "n{}->n{}[label=\"{}\"{}];",
                    id, out_index, distance, color
                )?;
            }
            // for update in &node.updates {
            //     write!(f, "n{}->n{}[style=dotted];", id, update)?;
            // }
        }
        f.write_str("}\n")
    }
}
