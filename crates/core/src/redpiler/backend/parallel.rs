//! The direct backend does not do code generation and operates on the `CompileNode` graph directly

use super::{JITBackend, NonMaxU8};
use crate::redpiler::compile_graph::{CompileGraph, LinkType, NodeIdx};
use crate::redpiler::{block_powered_mut, bool_to_ss};
use crate::world::World;
use itertools::Itertools;
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::blocks::{Block, ComparatorMode};
use mchprs_blocks::BlockPos;
use mchprs_world::{TickEntry, TickPriority};
use petgraph::visit::EdgeRef;
use petgraph::Direction;
use rayon::prelude::{IntoParallelRefMutIterator, IntoParallelRefIterator, ParallelIterator, IndexedParallelIterator, ParallelDrainRange};
use rayon::slice::ParallelSlice;
use rustc_hash::{FxHashMap, FxHashSet};
use smallvec::SmallVec;
use std::num::NonZeroU8;
use std::sync::atomic::AtomicU8;
use std::{fmt, mem};
use tracing::{debug, trace, warn};


#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct NodeId(u32);

impl NodeId {
    fn from_index(idx: usize) -> NodeId {
        NodeId(idx.try_into().unwrap())
    }

    fn index(&self) -> usize {
        self.0 as usize
    }
}

#[derive(Debug, Default)]
struct FinalGraphStats {
    update_link_count: usize,
    side_link_count: usize,
    default_link_count: usize,
    nodes_bytes: usize,
}

#[derive(Debug, Clone, Copy)]
struct DirectLink {
    weight: u8,
    to: NodeId,
}

#[derive(Debug, Clone, Copy)]
enum NodeType {
    Repeater(u8),
    /// A non-locking repeater
    SimpleRepeater(u8),
    Torch,
    Comparator(ComparatorMode),
    Lamp,
    Button,
    Lever,
    PressurePlate,
    Trapdoor,
    Wire,
    Constant,
}

// struct is 128 bytes to fit nicely into cachelines
// which are usualy 64 bytes, it can vary but is almost always a power of 2
#[derive(Debug, Clone)]
#[repr(align(128))]
pub struct Node {
    ty: NodeType,
    default_inputs: SmallVec<[DirectLink; 7]>,
    side_inputs: SmallVec<[DirectLink; 2]>,
    updates: SmallVec<[NodeId; 4]>,
    comparator_far_input: Option<NonMaxU8>,

    output_power: Power,
    locked: bool,

    is_io: bool,
}

type Power = u8;

impl Node {
    fn from_compile_node(
        graph: &CompileGraph,
        node_idx: NodeIdx,
        nodes_len: usize,
        nodes_map: &FxHashMap<NodeIdx, usize>,
        stats: &mut FinalGraphStats,
    ) -> Self {
        let node = &graph[node_idx];

        let mut default_inputs = SmallVec::new();
        let mut side_inputs = SmallVec::new();
        for edge in graph.edges_directed(node_idx, Direction::Incoming) {
            let idx = nodes_map[&edge.source()];
            assert!(idx < nodes_len);
            let idx = unsafe {
                // Safety: bounds checked
                NodeId::from_index(idx)
            };
            let link = DirectLink {
                to: idx,
                weight: edge.weight().ss,
            };
            match edge.weight().ty {
                LinkType::Default => default_inputs.push(link),
                LinkType::Side => side_inputs.push(link),
            }
        }
        stats.default_link_count += default_inputs.len();
        stats.side_link_count += side_inputs.len();

        use crate::redpiler::compile_graph::NodeType as CNodeType;
        let updates: SmallVec<[NodeId; 4]> = if node.ty != CNodeType::Constant {
            graph
                .neighbors_directed(node_idx, Direction::Outgoing)
                .map(|idx| unsafe {
                    let idx = nodes_map[&idx];
                    assert!(idx < nodes_len);
                    // Safety: bounds checked
                    NodeId::from_index(idx)
                })
                .collect()
        } else {
            SmallVec::new()
        };
        stats.update_link_count += updates.len();

        let ty = match node.ty {
            CNodeType::Repeater(delay) => {
                if side_inputs.is_empty() {
                    NodeType::SimpleRepeater(delay)
                } else {
                    NodeType::Repeater(delay)
                }
            }
            CNodeType::Torch => NodeType::Torch,
            CNodeType::Comparator(mode) => NodeType::Comparator(mode),
            CNodeType::Lamp => NodeType::Lamp,
            CNodeType::Button => NodeType::Button,
            CNodeType::Lever => NodeType::Lever,
            CNodeType::PressurePlate => NodeType::PressurePlate,
            CNodeType::Trapdoor => NodeType::Trapdoor,
            CNodeType::Wire => NodeType::Wire,
            CNodeType::Constant => NodeType::Constant,
            CNodeType::Buffer(delay) => panic!("ParallelBackend does not support buffers"),
        };

        Node {
            ty,
            default_inputs,
            side_inputs,
            updates,
            output_power: node.state.output_strength,
            locked: node.state.repeater_locked,
            comparator_far_input: node.comparator_far_input.map(|n| NonMaxU8::new(n).unwrap()),
            is_io: node.is_input || node.is_output,
        }
    }
}

#[derive(Default)]
struct TickScheduler {
    queue_deque: [Vec<NodeId>; 16],
    pos: usize,
}

impl TickScheduler {
    fn reset<W: World>(&mut self, world: &mut W, blocks: &[Option<(BlockPos, Block)>]) {
        for (idx, queue) in self.queue_deque.iter().enumerate() {
            let delay = if self.pos >= idx { idx + 16 } else { idx } - self.pos;
                for node in queue {
                    let Some((pos, _)) = blocks[node.index()] else {
                        warn!("Cannot schedule tick for node {:?} because block information is missing", node);
                        continue;
                    };
                    world.schedule_tick(pos, delay as u32, TickPriority::Normal);
                }
        }
        for queue in self.queue_deque.iter_mut() {
            queue.clear();
        }
    }

    fn schedule_tick(&mut self, node: NodeId, delay: usize) {
        self.queue_deque[(self.pos + delay) & 15].push(node);
    }

    fn queues_this_tick(&mut self) -> Vec<NodeId> {
        self.pos = (self.pos + 1) & 15;
        mem::take(&mut self.queue_deque[self.pos])
    }

    fn end_tick(&mut self, mut queue: Vec<NodeId>) {
        queue.clear();
        self.queue_deque[self.pos] = queue;
    }
}

#[derive(Default)]
pub struct ParallelBackend {
    nodes: Box<[Node]>,
    output_powers: Box<[Power]>,
    blocks: Vec<Option<(BlockPos, Block)>>,
    pos_map: FxHashMap<BlockPos, NodeId>,
    scheduler: TickScheduler,
}

impl ParallelBackend {
    fn schedule_tick(&mut self, node_id: NodeId, delay: usize) {
        self.scheduler.schedule_tick(node_id, delay);
    }

    fn set_node(&mut self, node_id: NodeId, powered: bool, new_power: u8) {
        let node = &mut self.nodes[node_id];
        node.changed = true;
        node.powered = powered;
        node.output_power = new_power;
        for i in 0..node.updates.len() {
            let update = self.nodes[node_id].updates[i];
            update_node(&mut self.scheduler, &mut self.nodes, update);
        }
    }
}

#[derive(Copy, Clone)]
struct PowerPointer(*mut Power);
unsafe impl Send for PowerPointer {}
unsafe impl Sync for PowerPointer {}

#[derive(Copy, Clone)]
struct NodePointer(*mut Node);
unsafe impl Send for NodePointer {}
unsafe impl Sync for NodePointer {}

impl JITBackend for ParallelBackend {
    fn inspect(&mut self, pos: BlockPos) -> Option<(bool, u8)> {
        let Some(node_id) = self.pos_map.get(&pos) else {
            debug!("could not find node at pos {}", pos);
            return None;
        };

        let node = &self.nodes[*node_id];
        debug!("Node {:?}: {:#?}", node_id, node);
        return Some((node.powered, node.output_power));
    }

    fn reset<W: World>(&mut self, world: &mut W, io_only: bool) {
        self.scheduler.reset(world, &self.blocks);

        let nodes = std::mem::take(&mut self.nodes);

        for (i, node) in nodes.into_inner().iter().enumerate() {
            let Some((pos, block)) = self.blocks[i] else {
                continue;
            };
            if matches!(node.ty, NodeType::Comparator(_)) {
                let block_entity = BlockEntity::Comparator {
                    output_strength: node.output_power,
                };
                world.set_block_entity(pos, block_entity);
            }

            if io_only && !node.is_io {
                world.set_block(pos, block);
            }
        }

        self.pos_map.clear();
    }

    fn on_use_block(&mut self, pos: BlockPos) {
        let node_id = self.pos_map[&pos];
        let node = &self.nodes[node_id];
        match node.ty {
            NodeType::Button => {
                if node.powered {
                    return;
                }
                self.schedule_tick(node_id, 10);
                self.set_node(node_id, true, 15);
            }
            NodeType::Lever => {
                self.set_node(node_id, !node.powered, bool_to_ss(!node.powered));
            }
            _ => warn!("Tried to use a {:?} redpiler node", node.ty),
        }
    }

    fn set_pressure_plate(&mut self, pos: BlockPos, powered: bool) {
        let node_id = self.pos_map[&pos];
        let node = &self.nodes[node_id];
        match node.ty {
            NodeType::PressurePlate => {
                self.set_node(node_id, powered, bool_to_ss(powered));
            }
            _ => warn!("Tried to set pressure plate state for a {:?}", node.ty),
        }
    }

    fn tick(&mut self) {
        let mut queues = self.scheduler.queues_this_tick();

        let power_ptr = PowerPointer(self.output_powers.as_mut_ptr());
        let update_set: FxHashSet<NodeId> = queues.par_iter().with_min_len(100).flat_map_iter(move |&node_id| {
            let node = &self.nodes[node_id.index()];
            let old_power = node.output_power;
            let new_power = unsafe { &mut *power_ptr.0.add(node_id.index()) };

            match node.ty {
                NodeType::Repeater(delay) => {
                    if !node.locked {
                        let should_be_powered = nodes_get_bool_input(node.default_inputs, &self.nodes);
                    if old_power > 0 && !should_be_powered {
                        self.set_node(node_id, false, 0);
                    } else if old_power == 0 {
                        self.set_node(node_id, true, 15);
                        if !should_be_powered {
                            //schedule_tick(
                            //    &mut self.scheduler,
                            //    node_id,
                            //    node,
                            //    delay as usize,
                            //);
                        }
                    }
                    }
                }
                NodeType::SimpleRepeater(delay) => {
                    let should_be_powered = nodes_get_bool_input(node.default_inputs, &self.nodes);
                    if old_power > 0 && !should_be_powered {
                        self.set_node(node_id, false, 0);
                    } else if old_power == 0 {
                        self.set_node(node_id, true, 15);
                        if !should_be_powered {
                            //schedule_tick(
                            //    &mut self.scheduler,
                            //    node_id,
                            //    node,
                            //    delay as usize,
                            //);
                        }
                    }
                }
                NodeType::Torch => {
                    let should_be_off = nodes_get_bool_input(node.default_inputs, &self.nodes);
                    let lit = old_power > 0;
                    if lit && should_be_off {
                        *new_power = 0;
                    } else if !lit && !should_be_off {
                        *new_power = 15;
                    }
                }
                NodeType::Comparator(mode) => {
                    let mut input_power = nodes_get_input(node.default_inputs, &self.nodes);
                    let mut side_power = nodes_get_input(node.default_inputs, &self.nodes);
                    if input_power < 15 {
                        if let Some(far_override) = node.comparator_far_input {
                            input_power = far_override.get();
                        }
                    }
                    *new_power = calculate_comparator_output(mode, input_power, side_power);
                }
                NodeType::Lamp => {
                    *new_power = bool_to_ss(nodes_get_bool_input(node.default_inputs, &self.nodes));
                }
                NodeType::Button => {
                    *new_power = 0;
                }
                _ => warn!("Node {:?} should not be ticked!", node.ty),
            }
            if *new_power != old_power {
                node.updates
            } else {
                SmallVec::new()
            }
        }).collect();

        let node_ptr = NodePointer(self.nodes.as_mut_ptr());
        let tick_set: FxHashSet<NodeId> = update_set.par_iter().filter_map(move |&node_id| {
            let node = unsafe { &mut *node_ptr.0.add(node_id.index()) };
            node.output_power = self.output_powers[node_id.index()];

            match node.ty {
                NodeType::Repeater(delay) => {
                    let (input_power, side_input_power) = powers_get_all_input(node, nodes);
                    let node = &mut nodes[node_id];
                    let should_be_locked = side_input_power > 0;
                    if !node.locked && should_be_locked {
                        set_node_locked(node, true);
                    } else if node.locked && !should_be_locked {
                        set_node_locked(node, false);
                    }
        
                    if !node.locked && !node.pending_tick {
                        let should_be_powered = input_power > 0;
                        if should_be_powered != node.powered {
                            schedule_tick(scheduler, node_id, node, delay as usize);
                        }
                    }
                }
                NodeType::SimpleRepeater(delay) => {
                    if node.pending_tick {
                        return;
                    }
                    let should_be_powered = get_bool_input(node, nodes);
                    if node.powered != should_be_powered {
                        let node = &mut nodes[node_id];
                        schedule_tick(scheduler, node_id, node, delay as usize);
                    }
                }
                NodeType::Torch => {
                    let should_be_off = powers_get_bool_input(node, &self.output_powers);
                    let lit = node.powered;
                    if lit == should_be_off {
                        let node = &mut nodes[node_id];
                        schedule_tick(scheduler, node_id, node, 1);
                    }
                }
                NodeType::Comparator(mode) => {
                    if node.pending_tick {
                        return;
                    }
                    let (mut input_power, side_input_power) = get_all_input(node, nodes);
                    if let Some(far_override) = node.comparator_far_input {
                        if input_power < 15 {
                            input_power = far_override.get();
                        }
                    }
                    let old_strength = node.output_power;
                    let output_power = calculate_comparator_output(mode, input_power, side_input_power);
                    if output_power != old_strength {
                        let node = &mut nodes[node_id];
                        schedule_tick(scheduler, node_id, node, 1);
                    }
                }
                NodeType::Lamp => {
                    let should_be_lit = get_bool_input(node, nodes);
                    let lit = node.powered;
                    let node = &mut nodes[node_id];
                    if lit && !should_be_lit {
                        schedule_tick(scheduler, node_id, node, 2);
                    } else if !lit && should_be_lit {
                        set_node(node, true);
                    }
                }
                NodeType::Trapdoor => {
                    let should_be_powered = get_bool_input(node, nodes);
                    if node.powered != should_be_powered {
                        let node = &mut nodes[node_id];
                        set_node(node, should_be_powered);
                    }
                }
                NodeType::Wire => {
                    let (input_power, _) = get_all_input(node, nodes);
                    if node.output_power != input_power {
                        let node = &mut nodes[node_id];
                        node.output_power = input_power;
                        node.changed = true;
                    }
                }
                _ => panic!("Node {:?} should not be updated!", node),
            }
        }).collect();

        self.scheduler.end_tick(queues);
    }

    fn compile(&mut self, graph: CompileGraph, ticks: Vec<TickEntry>) {
        let mut nodes_map =
            FxHashMap::with_capacity_and_hasher(graph.node_count(), Default::default());
        for node in graph.node_indices() {
            nodes_map.insert(node, nodes_map.len());
        }
        let nodes_len = nodes_map.len();

        let mut stats = FinalGraphStats::default();
        self.nodes = graph
            .node_indices()
            .map(|idx| Node::from_compile_node(&graph, idx, nodes_len, &nodes_map, &mut stats))
            .collect();
        stats.nodes_bytes = nodes_len * std::mem::size_of::<Node>();
        trace!("{:#?}", stats);

        self.blocks = graph
            .node_weights()
            .map(|node| node.block.map(|(pos, id)| (pos, Block::from_id(id))))
            .collect();

        for i in 0..self.blocks.len() {
            if let Some((pos, _)) = self.blocks[i] {
                self.pos_map.insert(pos, NodeId::from_index(i));
            }
        }

        for entry in ticks {
            if let Some(node_id) = self.pos_map.get(&entry.pos) {
                self.scheduler
                    .schedule_tick(*node_id, entry.ticks_left as usize);
                self.nodes[node_id.index()].pending_tick = true;
            }
        }
        // Dot file output
        // println!("{}", self);
    }

    fn flush<W: World>(&mut self, world: &mut W, io_only: bool) {
        for (i, node) in self.nodes.inner_mut().iter_mut().enumerate() {
            let Some((pos, block)) = &mut self.blocks[i] else {
                continue;
            };
            if node.changed && (!io_only || node.is_io) {
                if let Some(powered) = block_powered_mut(block) {
                    *powered = node.powered
                }
                if let Block::RedstoneWire { wire, .. } = block {
                    wire.power = node.output_power
                };
                if let Block::RedstoneRepeater { repeater } = block {
                    repeater.locked = node.locked;
                }
                world.set_block(*pos, *block);
            }
            node.changed = false;
        }
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
) {
    node.pending_tick = true;
    scheduler.schedule_tick(node_id, delay);
}

fn powers_link_strength(link: DirectLink, powers: &[Power]) -> u8 {
    powers[link.to.index()].saturating_sub(link.weight)
}

fn powers_get_bool_input(mut inputs: impl IntoIterator<Item = DirectLink>, powers: &[Power]) -> bool {
    inputs.into_iter().any(|link| link_strength(link, powers) > 0)
}

fn powers_get_input(mut inputs: impl IntoIterator<Item = DirectLink>, powers: &[Power]) -> u8 {
    inputs.into_iter()
        .map(|link| link_strength(link, powers))
        .max()
        .unwrap_or(0)
}

fn nodes_link_strength(link: DirectLink, nodes: &[Node]) -> u8 {
    nodes[link.to.index()].output_power.saturating_sub(link.weight)
}

fn nodes_get_bool_input(mut inputs: impl IntoIterator<Item = DirectLink>, nodes: &[Node]) -> bool {
    inputs.into_iter().any(|link| link_strength(link, nodes) > 0)
}

fn nodes_get_input(mut inputs: impl IntoIterator<Item = DirectLink>, nodes: &[Node]) -> u8 {
    inputs.into_iter()
        .map(|link| link_strength(link, nodes))
        .max()
        .unwrap_or(0)
}


#[inline(always)]
fn update_node(scheduler: &mut TickScheduler, nodes: &mut Nodes, node_id: NodeId) {
    let node = &nodes[node_id];

    match node.ty {
        NodeType::Repeater(delay) => {
            let (input_power, side_input_power) = get_all_input(node, nodes);
            let node = &mut nodes[node_id];
            let should_be_locked = side_input_power > 0;
            if !node.locked && should_be_locked {
                set_node_locked(node, true);
            } else if node.locked && !should_be_locked {
                set_node_locked(node, false);
            }

            if !node.locked && !node.pending_tick {
                let should_be_powered = input_power > 0;
                if should_be_powered != node.powered {
                    schedule_tick(scheduler, node_id, node, delay as usize);
                }
            }
        }
        NodeType::SimpleRepeater(delay) => {
            if node.pending_tick {
                return;
            }
            let should_be_powered = get_bool_input(node, nodes);
            if node.powered != should_be_powered {
                let node = &mut nodes[node_id];
                schedule_tick(scheduler, node_id, node, delay as usize);
            }
        }
        NodeType::Torch => {
            if node.pending_tick {
                return;
            }
            let should_be_off = get_bool_input(node, nodes);
            let lit = node.powered;
            if lit == should_be_off {
                let node = &mut nodes[node_id];
                schedule_tick(scheduler, node_id, node, 1);
            }
        }
        NodeType::Comparator(mode) => {
            if node.pending_tick {
                return;
            }
            let (mut input_power, side_input_power) = get_all_input(node, nodes);
            if let Some(far_override) = node.comparator_far_input {
                if input_power < 15 {
                    input_power = far_override.get();
                }
            }
            let old_strength = node.output_power;
            let output_power = calculate_comparator_output(mode, input_power, side_input_power);
            if output_power != old_strength {
                let node = &mut nodes[node_id];
                schedule_tick(scheduler, node_id, node, 1);
            }
        }
        NodeType::Lamp => {
            let should_be_lit = get_bool_input(node, nodes);
            let lit = node.powered;
            let node = &mut nodes[node_id];
            if lit && !should_be_lit {
                schedule_tick(scheduler, node_id, node, 2);
            } else if !lit && should_be_lit {
                set_node(node, true);
            }
        }
        NodeType::Trapdoor => {
            let should_be_powered = get_bool_input(node, nodes);
            if node.powered != should_be_powered {
                let node = &mut nodes[node_id];
                set_node(node, should_be_powered);
            }
        }
        NodeType::Wire => {
            let (input_power, _) = get_all_input(node, nodes);
            if node.output_power != input_power {
                let node = &mut nodes[node_id];
                node.output_power = input_power;
                node.changed = true;
            }
        }
        _ => panic!("Node {:?} should not be updated!", node),
    }
}

impl fmt::Display for ParallelBackend {
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
            let all_inputs = node
                .default_inputs
                .iter()
                .map(|link| (LinkType::Default, link))
                .chain(node.side_inputs.iter().map(|link| (LinkType::Side, link)));
            for (link_type, link) in all_inputs {
                let color = match link_type {
                    LinkType::Default => "",
                    LinkType::Side => ",color=\"blue\"",
                };
                write!(
                    f,
                    "n{}->n{}[label=\"{}\"{}];",
                    link.to.index(),
                    id,
                    link.weight,
                    color
                )?;
            }
            // for update in &node.updates {
            //     write!(f, "n{}->n{}[style=dotted];", id, update)?;
            // }
        }
        f.write_str("}\n")
    }
}

fn calculate_comparator_output(mode: ComparatorMode, input_strength: u8, power_on_sides: u8) -> u8 {
    match mode {
        ComparatorMode::Compare => {
            if input_strength >= power_on_sides {
                input_strength
            } else {
                0
            }
        }
        ComparatorMode::Subtract => input_strength.saturating_sub(power_on_sides),
    }
}
