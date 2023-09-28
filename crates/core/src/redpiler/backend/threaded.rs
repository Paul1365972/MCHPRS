//! The direct backend does not do code generation and operates on the `CompileNode` graph directly

use super::JITBackend;
use crate::redpiler::compile_graph::{self, CompileGraph, LinkType, NodeIdx};
use crate::redpiler::{block_powered_mut, bool_to_ss};
use crate::world::World;
use crossbeam::channel::{Receiver, Sender};
use itertools::Itertools;
use mchprs_blocks::blocks::{Block, ComparatorMode};
use mchprs_blocks::BlockPos;
use mchprs_world::{TickEntry, TickPriority};
use nodes::{NodeId, Nodes};
use petgraph::visit::EdgeRef;
use petgraph::Direction;
use rustc_hash::FxHashMap;
use smallvec::SmallVec;
use std::thread::JoinHandle;
use std::{mem, thread};
use tracing::{trace, warn};

#[derive(Debug, Default)]
struct FinalGraphStats {
    update_link_count: usize,
    side_link_count: usize,
    default_link_count: usize,
    nodes_bytes: usize,
}

mod nodes {
    use super::Node;
    use std::ops::{Index, IndexMut};

    #[derive(Debug, Copy, Clone, PartialEq, Eq)]
    pub struct NodeId(u32);

    impl NodeId {
        pub fn index(self) -> usize {
            self.0 as usize
        }

        /// Safety: index must be within bounds of nodes array
        pub fn from_index(index: usize) -> NodeId {
            NodeId(index as u32)
        }
    }

    // This is Pretty Bad:tm: because one can create a NodeId using another instance of Nodes,
    // but at least some type system protection is better than none.
    #[derive(Default)]
    pub struct Nodes {
        nodes: Box<[Node]>,
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

        pub fn into_inner(self) -> Box<[Node]> {
            self.nodes
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
}

#[derive(Debug, Clone, Copy)]
struct DirectLink {
    weight: u8,
    to: NodeId,
}

#[derive(Debug, Clone, Copy)]
enum NodeType {
    Repeater {
        delay: u8,
        facing_diode: bool,
        locked: bool,
    },
    /// A non-locking repeater
    SimpleRepeater {
        delay: u8,
        facing_diode: bool,
    },
    Torch,
    Comparator {
        mode: ComparatorMode,
        comparator_far_input: Option<u8>,
        facing_diode: bool,
    },
    Lamp,
    Button,
    Lever,
    PressurePlate,
    Trapdoor,
    Wire,
    Constant,
    Buffer {
        delay: u8,
        buffer_state: u32,
        buffer_index: u8,
    },
    SubgraphInput,
    SubgraphOutput {
        channel_id: u16,
        node_id: NodeId,
    },
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
    is_io: bool,

    /// Powered or lit
    powered: bool,
    output_power: u8,
    changed: bool,
    pending_tick: bool,
}

impl Node {
    fn from_compile_node(
        graph: &CompileGraph,
        node_idx: NodeIdx,
        nodes_map: &FxHashMap<NodeIdx, (SubgraphId, NodeId)>,
        output_redirects: &mut Vec<SubgraphId>,
        stats: &mut FinalGraphStats,
    ) -> Self {
        let (sid, nid) = nodes_map[&node_idx];
        let node = &graph[node_idx];

        let mut default_inputs = SmallVec::new();
        let mut side_inputs = SmallVec::new();
        for edge in graph.edges_directed(node_idx, Direction::Incoming) {
            let link = DirectLink {
                to: nodes_map[&edge.source()].1,
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
                .map(|idx| nodes_map[&idx].1)
                .collect()
        } else {
            SmallVec::new()
        };
        stats.update_link_count += updates.len();

        let ty = match node.ty {
            CNodeType::Repeater(delay) => {
                if side_inputs.is_empty() {
                    NodeType::SimpleRepeater {
                        delay,
                        facing_diode: node.facing_diode,
                    }
                } else {
                    NodeType::Repeater {
                        delay,
                        facing_diode: node.facing_diode,
                        locked: node.state.repeater_locked,
                    }
                }
            }
            CNodeType::Torch => NodeType::Torch,
            CNodeType::Comparator(mode) => NodeType::Comparator {
                mode,
                comparator_far_input: node.comparator_far_input,
                facing_diode: node.facing_diode,
            },
            CNodeType::Lamp => NodeType::Lamp,
            CNodeType::Button => NodeType::Button,
            CNodeType::Lever => NodeType::Lever,
            CNodeType::PressurePlate => NodeType::PressurePlate,
            CNodeType::Trapdoor => NodeType::Trapdoor,
            CNodeType::Wire => NodeType::Wire,
            CNodeType::Constant => NodeType::Constant,
            CNodeType::Buffer(delay) => NodeType::Buffer {
                delay,
                buffer_state: node.state.output_strength as u32,
                buffer_index: 0,
            },
            CNodeType::SubgraphInput(_) => NodeType::SubgraphInput,
            CNodeType::SubgraphOutput(idx) => {
                let (subgraph_id, node_id) = nodes_map[&idx];
                let channel_id = if let Some(channel_id) =
                    output_redirects.iter().position(|&x| x == subgraph_id)
                {
                    channel_id
                } else {
                    output_redirects.push(subgraph_id);
                    output_redirects.len() - 1
                }
                .try_into()
                .unwrap();
                NodeType::SubgraphOutput {
                    channel_id,
                    node_id,
                }
            }
        };

        Node {
            ty,
            default_inputs,
            side_inputs,
            updates,
            powered: node.state.powered,
            output_power: node.state.output_strength,
            pending_tick: false,
            changed: false,
            is_io: node.is_input || node.is_output,
        }
    }
}

#[derive(Default, Clone)]
struct Queues([Vec<NodeId>; TickScheduler::NUM_PRIORITIES]);

impl Queues {
    fn drain_iter(&mut self) -> impl Iterator<Item = NodeId> + '_ {
        let [q0, q1, q2, q3] = &mut self.0;
        let [q0, q1, q2, q3] = [q0, q1, q2, q3].map(|q| q.drain(..));
        q0.chain(q1).chain(q2).chain(q3)
    }
}

#[derive(Default)]
struct TickScheduler {
    queues_deque: [Queues; 16],
    pos: usize,
}

impl TickScheduler {
    const NUM_PRIORITIES: usize = 4;

    fn reset<W: World>(&mut self, world: &mut W, blocks: &[Option<(BlockPos, Block)>]) {
        for (idx, queues) in self.queues_deque.iter().enumerate() {
            let delay = if self.pos >= idx { idx + 16 } else { idx } - self.pos;
            for (entries, priority) in queues.0.iter().zip(Self::priorities()) {
                for node in entries {
                    let Some((pos, _)) = blocks[node.index()] else {
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

    fn schedule_tick(&mut self, node: NodeId, delay: usize, priority: TickPriority) {
        self.queues_deque[(self.pos + delay) & 15].0[Self::priority_index(priority)].push(node);
    }

    fn queues_this_tick(&mut self) -> Queues {
        self.pos = (self.pos + 1) & 15;
        mem::take(&mut self.queues_deque[self.pos])
    }

    fn end_tick(&mut self, mut queues: Queues) {
        for queue in &mut queues.0 {
            queue.clear();
        }
        self.queues_deque[self.pos] = queues;
    }

    fn priorities() -> [TickPriority; Self::NUM_PRIORITIES] {
        [
            TickPriority::Highest,
            TickPriority::Higher,
            TickPriority::High,
            TickPriority::Normal,
        ]
    }

    fn priority_index(priority: TickPriority) -> usize {
        match priority {
            TickPriority::Highest => 0,
            TickPriority::Higher => 1,
            TickPriority::High => 2,
            TickPriority::Normal => 3,
        }
    }
}

#[derive(Default)]
pub struct ThreadedBackend {
    handles: Vec<JoinHandle<()>>,
    roots: Vec<Sender<Message>>,
    changes: Option<Receiver<Vec<(BlockPos, Block)>>>,
    pos_map: FxHashMap<BlockPos, (SubgraphId, NodeId)>,
}

enum Message {
    Updates(Vec<[Vec<(NodeId, u8)>; TickScheduler::NUM_PRIORITIES]>),
    UseBlock(SubgraphId, NodeId),
    PressurePlate(SubgraphId, NodeId, bool),
    Flush { io_only: bool },
    Close,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
struct SubgraphId(u32);

pub struct Subgraph {
    id: SubgraphId,
    inputs: Vec<Receiver<Message>>,
    outputs: Vec<Sender<Message>>,
    changes: Option<Sender<Vec<(BlockPos, Block)>>>,
    nodes: Nodes,
    scheduler: TickScheduler,
    blocks: Vec<Option<(BlockPos, Block)>>,
    output_redirects: Vec<SubgraphId>,
}

impl Subgraph {
    fn schedule_tick(&mut self, node_id: NodeId, delay: usize, priority: TickPriority) {
        self.scheduler.schedule_tick(node_id, delay, priority);
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

impl JITBackend for ThreadedBackend {
    fn compile(&mut self, graph: CompileGraph, ticks: Vec<TickEntry>) {
        let components = compile_graph::weakly_connected_components(&graph);

        let mut subgraphs = vec![];
        let (changes_sender, changes_receiver) = crossbeam::channel::unbounded();
        self.changes = Some(changes_receiver);

        let nodes_map: FxHashMap<NodeIdx, (SubgraphId, NodeId)> = components
            .iter()
            .enumerate()
            .flat_map(|(cid, component)| {
                component.iter().enumerate().map(move |(nid, idx)| {
                    (
                        *idx,
                        (SubgraphId(cid.try_into().unwrap()), NodeId::from_index(nid)),
                    )
                })
            })
            .collect();

        for component in components.iter() {
            let mut stats = FinalGraphStats::default();
            let mut nodes = vec![];
            let mut blocks = vec![];
            let mut output_redirects = vec![];
            for &idx in component.iter() {
                let (sid, nid) = nodes_map[&idx];
                let node = &graph[idx];

                if let Some((pos, id)) = node.block {
                    blocks.push(Some((pos, Block::from_id(id))));
                    self.pos_map.insert(pos, (sid, nid));
                }

                nodes.push(Node::from_compile_node(
                    &graph,
                    idx,
                    &nodes_map,
                    &mut output_redirects,
                    &mut stats,
                ));
            }

            stats.nodes_bytes = graph.node_count() * std::mem::size_of::<Node>();
            trace!("{:#?}", stats);
            subgraphs.push(Subgraph {
                id: SubgraphId(subgraphs.len().try_into().unwrap()),
                inputs: vec![],
                outputs: vec![],
                changes: Some(changes_sender.clone()),
                nodes: Nodes::new(nodes.into_boxed_slice()),
                scheduler: TickScheduler::default(),
                blocks,
                output_redirects,
            });
        }

        for sid in 0..subgraphs.len() {
            for redirect_sid in subgraphs[sid].output_redirects.clone() {
                let (sender, receiver) = crossbeam::channel::bounded(100);
                subgraphs[sid].outputs.push(sender);
                subgraphs[redirect_sid.0 as usize].inputs.push(receiver);
            }
        }
        for subgraph in subgraphs.iter_mut().filter(|s| s.inputs.is_empty()) {
            let (sender, receiver) = crossbeam::channel::bounded(100);
            self.roots.push(sender);
            subgraph.inputs.push(receiver);
        }

        for entry in ticks {
            if let Some((sid, nid)) = self.pos_map.get(&entry.pos) {
                let subgraph = &mut subgraphs[sid.0 as usize];
                subgraph.scheduler.schedule_tick(
                    *nid,
                    entry.ticks_left as usize,
                    entry.tick_priority,
                );
                subgraph.nodes[*nid].pending_tick = true;
            }
        }

        self.handles = subgraphs.into_iter().map(|mut subgraph| thread::spawn(move || subgraph.run())).collect();
    }

    fn tick(&mut self, ticks: u64) {
        let data = vec![Default::default(); ticks as usize];
        for root in self.roots.iter() {
            root.send(Message::Updates(data.clone())).unwrap();
        }
    }

    fn on_use_block(&mut self, pos: BlockPos) {
        let (sid, nid) = self.pos_map[&pos];
        for root in self.roots.iter() {
            root.send(Message::UseBlock(sid, nid)).unwrap();
        }
    }

    fn set_pressure_plate(&mut self, pos: BlockPos, powered: bool) {
        let (sid, nid) = self.pos_map[&pos];
        for root in self.roots.iter() {
            root.send(Message::PressurePlate(sid, nid, powered)).unwrap();
        }
    }

    fn flush<W: World>(&mut self, world: &mut W, io_only: bool) {
        for root in self.roots.iter() {
            root.send(Message::Flush { io_only }).unwrap();
        }
        while let Ok(data) = self.changes.as_ref().unwrap().try_recv() {
            for (pos, block) in data {
                world.set_block(pos, block);
            }
        }
    }

    fn reset<W: World>(&mut self, world: &mut W, io_only: bool) {
        for root in self.roots.iter() {
            root.send(Message::Flush { io_only: false }).unwrap();
            root.send(Message::Close).unwrap();
        }
        self.handles.drain(..).for_each(|handle| handle.join().unwrap());
        // TODO: Changes messages may not have arrived yet here?
        for data in self.changes.as_ref().unwrap() {
            for (pos, block) in data {
                world.set_block(pos, block);
            }
        }
        self.changes = None;
    }

    #[doc = " Inspect block for debugging"]
    fn inspect(&mut self, pos: BlockPos) -> Option<(bool, u8)> {
        todo!()
    }
}

impl Subgraph {
    fn run(&mut self) {
        let mut inputs = self.inputs.iter();
        let msg = inputs.next().unwrap().recv().unwrap();
        match msg {
            Message::Updates(data) => {
                let mut updates = data;
                for other in inputs {
                    let other = other.recv().unwrap();
                    if let Message::Updates(data) = other {
                        for (index, datum) in data.into_iter().enumerate() {
                            for (prio, entries) in datum.into_iter().enumerate() {
                                updates[index][prio].extend(entries);
                            }
                        }
                    } else {
                        panic!("Differing message types");
                    }
                }
                self.tick(updates);
            }
            Message::UseBlock(sid, node_id) => {
                for other in inputs {
                    let other = other.recv().unwrap();
                    assert!(
                        matches!(other, Message::UseBlock(o_sid, o_node_id) if o_sid == sid && o_node_id == node_id)
                    );
                }

                if sid == self.id {
                    let node = &self.nodes[node_id];
                    match node.ty {
                        NodeType::Button => {
                            if node.powered {
                                return;
                            }
                            self.schedule_tick(node_id, 10, TickPriority::Normal);
                            self.set_node(node_id, true, 15);
                        }
                        NodeType::Lever => {
                            self.set_node(node_id, !node.powered, bool_to_ss(!node.powered));
                        }
                        _ => warn!("Tried to use a {:?} redpiler node", node.ty),
                    }
                }
                for ouput in self.outputs.iter() {
                    ouput.send(Message::UseBlock(sid, node_id)).unwrap();
                }
            }
            Message::PressurePlate(sid, node_id, powered) => {
                for other in inputs {
                    let other = other.recv().unwrap();
                    assert!(
                        matches!(other, Message::PressurePlate(o_sid, o_node_id, o_powered) if o_sid == sid && o_node_id == node_id && o_powered == powered)
                    );
                }
                let node = &self.nodes[node_id];
                match node.ty {
                    NodeType::PressurePlate => {
                        self.set_node(node_id, powered, bool_to_ss(powered));
                    }
                    _ => warn!("Tried to set pressure plate state for a {:?}", node.ty),
                }
                for ouput in self.outputs.iter() {
                    ouput.send(Message::PressurePlate(sid, node_id, powered)).unwrap();
                }
            }
            Message::Flush { io_only } => {
                for other in inputs {
                    let other = other.recv().unwrap();
                    assert!(
                        matches!(other, Message::Flush { io_only: o_io_only } if o_io_only == io_only)
                    );
                }
                let mut changes = vec![];
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
                            repeater.locked = match node.ty {
                                NodeType::Repeater {
                                    delay: _,
                                    facing_diode: _,
                                    locked,
                                } => locked,
                                NodeType::SimpleRepeater {
                                    delay: _,
                                    facing_diode: _,
                                } => false,
                                _ => panic!("Underlying block type is not repeater anymore"),
                            };
                        }
                        changes.push((*pos, *block));
                    }
                    node.changed = false;
                }
                for output in self.outputs.iter() {
                    output.send(Message::Flush { io_only }).unwrap();
                }
                self.changes.as_ref().unwrap().send(changes).unwrap();
            }
            Message::Close => {
                for other in inputs {
                    let other = other.recv().unwrap();
                    assert!(matches!(other, Message::Close));
                }

                for output in self.outputs.iter() {
                    output.send(Message::Close).unwrap();
                }
                self.inputs.clear();
                self.outputs.clear();
                self.changes = None;
            }
        }
    }

    fn tick(&mut self, bulk_updates: Vec<[Vec<(NodeId, u8)>; TickScheduler::NUM_PRIORITIES]>) {
        for bulk_update in bulk_updates {
            let queues = self.scheduler.queues_this_tick();
            for (updates, ticks) in bulk_update.into_iter().zip_eq(queues.0) {
                for (node_id, new_power) in updates {
                    self.set_node(node_id, new_power > 0, new_power);
                }
                for node_id in ticks {
                    self.nodes[node_id].pending_tick = false;
                    let node = &self.nodes[node_id];

                    match node.ty {
                        NodeType::Repeater {
                            delay,
                            facing_diode: _,
                            locked,
                        } => {
                            if locked {
                                continue;
                            }

                            let should_be_powered = get_bool_input(node, &self.nodes);
                            if node.powered && !should_be_powered {
                                self.set_node(node_id, false, 0);
                            } else if !node.powered {
                                self.set_node(node_id, true, 15);
                                if !should_be_powered {
                                    let node = &mut self.nodes[node_id];
                                    schedule_tick(
                                        &mut self.scheduler,
                                        node_id,
                                        node,
                                        delay as usize,
                                        TickPriority::Higher,
                                    );
                                }
                            }
                        }
                        NodeType::SimpleRepeater {
                            delay,
                            facing_diode: _,
                        } => {
                            let should_be_powered = get_bool_input(node, &self.nodes);
                            if node.powered && !should_be_powered {
                                self.set_node(node_id, false, 0);
                            } else if !node.powered {
                                self.set_node(node_id, true, 15);
                                if !should_be_powered {
                                    let node = &mut self.nodes[node_id];
                                    schedule_tick(
                                        &mut self.scheduler,
                                        node_id,
                                        node,
                                        delay as usize,
                                        TickPriority::Higher,
                                    );
                                }
                            }
                        }
                        NodeType::Torch => {
                            let should_be_off = get_bool_input(node, &self.nodes);
                            let lit = node.powered;
                            if lit && should_be_off {
                                self.set_node(node_id, false, 0);
                            } else if !lit && !should_be_off {
                                self.set_node(node_id, true, 15);
                            }
                        }
                        NodeType::Comparator {
                            mode,
                            comparator_far_input,
                            facing_diode: _,
                        } => {
                            let (mut input_power, side_input_power) =
                                get_all_input(node, &self.nodes);
                            if let Some(far_override) = comparator_far_input {
                                if input_power < 15 {
                                    input_power = far_override;
                                }
                            }
                            let old_strength = node.output_power;
                            let new_strength =
                                calculate_comparator_output(mode, input_power, side_input_power);
                            if new_strength != old_strength {
                                self.set_node(node_id, new_strength > 0, new_strength);
                            }
                        }
                        NodeType::Lamp => {
                            let should_be_lit = get_bool_input(node, &self.nodes);
                            if node.powered && !should_be_lit {
                                self.set_node(node_id, false, 0);
                            }
                        }
                        NodeType::Button => {
                            if node.powered {
                                self.set_node(node_id, false, 0);
                            }
                        }
                        NodeType::Buffer {
                            delay,
                            buffer_state,
                            buffer_index,
                        } => {
                            let node = &mut self.nodes[node_id];
                            node.ty = NodeType::Buffer {
                                delay,
                                buffer_state: buffer_state >> 4,
                                buffer_index: buffer_index - 1,
                            };

                            let new_strength = (buffer_state >> 4) as u8 & 0b1111;
                            if new_strength != node.output_power {
                                self.set_node(node_id, new_strength > 0, new_strength);
                            }
                        }
                        _ => warn!("Node {:?} should not be ticked!", node.ty),
                    }
                }

                self.scheduler.end_tick(Queues::default());
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
    node_id: NodeId,
    node: &mut Node,
    delay: usize,
    priority: TickPriority,
) {
    node.pending_tick = true;
    scheduler.schedule_tick(node_id, delay, priority);
}

fn link_strength(link: DirectLink, nodes: &Nodes) -> u8 {
    nodes[link.to].output_power.saturating_sub(link.weight)
}

fn get_bool_input(node: &Node, nodes: &Nodes) -> bool {
    node.default_inputs
        .iter()
        .copied()
        .any(|link| link_strength(link, nodes) > 0)
}

fn get_all_input(node: &Node, nodes: &Nodes) -> (u8, u8) {
    let input_power = node
        .default_inputs
        .iter()
        .copied()
        .map(|link| link_strength(link, nodes))
        .max()
        .unwrap_or(0);

    let side_input_power = node
        .side_inputs
        .iter()
        .copied()
        .map(|link| link_strength(link, nodes))
        .max()
        .unwrap_or(0);

    (input_power, side_input_power)
}

#[inline(always)]
fn update_node(scheduler: &mut TickScheduler, nodes: &mut Nodes, node_id: NodeId) {
    let node = &nodes[node_id];

    match node.ty {
        NodeType::Repeater {
            delay,
            facing_diode,
            mut locked,
        } => {
            let (input_power, side_input_power) = get_all_input(node, nodes);
            let node = &mut nodes[node_id];
            let should_be_locked = side_input_power > 0;
            if !locked && should_be_locked {
                locked = true;
                node.changed = true;
            } else if locked && !should_be_locked {
                locked = false;
                node.changed = true;
            }

            if !locked && !node.pending_tick {
                let should_be_powered = input_power > 0;
                if should_be_powered != node.powered {
                    let priority = if facing_diode {
                        TickPriority::Highest
                    } else if !should_be_powered {
                        TickPriority::Higher
                    } else {
                        TickPriority::High
                    };
                    schedule_tick(scheduler, node_id, node, delay as usize, priority);
                }
            }
            node.ty = NodeType::Repeater {
                delay,
                facing_diode,
                locked,
            };
        }
        NodeType::SimpleRepeater {
            delay,
            facing_diode,
        } => {
            if node.pending_tick {
                return;
            }
            let should_be_powered = get_bool_input(node, nodes);
            if node.powered != should_be_powered {
                let priority = if facing_diode {
                    TickPriority::Highest
                } else if !should_be_powered {
                    TickPriority::Higher
                } else {
                    TickPriority::High
                };
                let node = &mut nodes[node_id];
                schedule_tick(scheduler, node_id, node, delay as usize, priority);
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
                schedule_tick(scheduler, node_id, node, 1, TickPriority::Normal);
            }
        }
        NodeType::Comparator {
            mode,
            comparator_far_input,
            facing_diode,
        } => {
            if node.pending_tick {
                return;
            }
            let (mut input_power, side_input_power) = get_all_input(node, nodes);
            if let Some(far_override) = comparator_far_input {
                if input_power < 15 {
                    input_power = far_override;
                }
            }
            let old_strength = node.output_power;
            let output_power = calculate_comparator_output(mode, input_power, side_input_power);
            if output_power != old_strength {
                let priority = if facing_diode {
                    TickPriority::High
                } else {
                    TickPriority::Normal
                };
                let node = &mut nodes[node_id];
                schedule_tick(scheduler, node_id, node, 1, priority);
            }
        }
        NodeType::Lamp => {
            let should_be_lit = get_bool_input(node, nodes);
            let lit = node.powered;
            let node = &mut nodes[node_id];
            if lit && !should_be_lit {
                schedule_tick(scheduler, node_id, node, 2, TickPriority::Normal);
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
        NodeType::Buffer {
            delay,
            mut buffer_state,
            mut buffer_index,
        } => {
            let (new_strength, _) = get_all_input(node, nodes);
            let old_strength = (buffer_state >> (buffer_index * 4)) as u8 & 0b1111;
            if new_strength != old_strength {
                let node = &mut nodes[node_id];
                buffer_index += 1;
                buffer_state |= (new_strength as u32) << (buffer_index * 4);
                schedule_tick(
                    scheduler,
                    node_id,
                    node,
                    delay as usize,
                    TickPriority::Normal,
                );
                node.ty = NodeType::Buffer {
                    delay,
                    buffer_state,
                    buffer_index,
                };
            }
        }
        _ => panic!("Node {:?} should not be updated!", node),
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
