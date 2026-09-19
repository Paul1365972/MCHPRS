//! The direct backend lowers the compile graph to nodes and interprets their updates and ticks.

mod compile;
mod node;
mod scheduler;
mod tick;
mod update;

use std::{
    fmt::{self, Write},
    sync::Arc,
};

use mchprs_blocks::{
    block_entities::BlockEntity,
    blocks::{Block, ComparatorMode, Instrument},
    BlockPos,
};
use mchprs_redstone::noteblock;
use mchprs_world::{TickEntry, TickPriority, World};
use rustc_hash::FxHashMap;
use smallvec::SmallVec;
use tracing::{debug, warn};

use self::node::{ForwardLinkRange, ForwardLinks, GateKind, Node, NodeId, NodeType, Nodes};
use self::scheduler::TickScheduler;
use super::JITBackend;
use crate::compile_graph::{CompileGraph, SignalStrength};
use crate::{block_powered_mut, CompilerOptions, TaskMonitor};

enum Event {
    NoteBlockPlay { noteblock_id: u16 },
}

#[derive(Default)]
pub struct DirectBackend {
    nodes: Nodes,
    forward_links: ForwardLinks,
    blocks: Vec<SmallVec<[(BlockPos, Block); 1]>>,
    pos_map: FxHashMap<BlockPos, NodeId>,
    scheduler: TickScheduler,
    events: Vec<Event>,
    noteblock_info: Vec<(SmallVec<[BlockPos; 1]>, Instrument, u8)>,
}

impl DirectBackend {
    fn flush_events<W: World>(&mut self, world: &mut W) {
        for event in self.events.drain(..) {
            match event {
                Event::NoteBlockPlay { noteblock_id } => {
                    let (positions, instrument, note) = &self.noteblock_info[noteblock_id as usize];
                    for pos in positions.iter().copied() {
                        noteblock::play_note(world, pos, *instrument, *note);
                    }
                }
            }
        }
    }

    fn schedule_tick(&mut self, node_id: NodeId, delay: usize, priority: TickPriority) {
        let node = &mut self.nodes[node_id];
        debug_assert!(!node.pending_tick);
        node.pending_tick = true;
        self.scheduler.schedule(node_id, delay, priority);
    }

    fn set_power_and_propagate(&mut self, node_id: NodeId, power: SignalStrength) {
        let node = &self.nodes[node_id];
        let old_power = node.power;
        let links = node.links;
        self.nodes.set_power(node_id, power);

        match (old_power, power) {
            (SignalStrength::ZERO, SignalStrength::MAX) => self.propagate(links, |weight| {
                Some((
                    SignalStrength::ZERO,
                    SignalStrength::MAX.saturating_sub(weight),
                ))
            }),
            (SignalStrength::MAX, SignalStrength::ZERO) => self.propagate(links, |weight| {
                Some((
                    SignalStrength::MAX.saturating_sub(weight),
                    SignalStrength::ZERO,
                ))
            }),
            _ => self.propagate(links, |weight| {
                let old_input = old_power.saturating_sub(weight);
                let new_input = power.saturating_sub(weight);
                (old_input != new_input).then_some((old_input, new_input))
            }),
        }
    }

    #[inline(always)]
    fn propagate(
        &mut self,
        links: ForwardLinkRange,
        input_change: impl Fn(u8) -> Option<(SignalStrength, SignalStrength)>,
    ) {
        for index in links.iter() {
            let link = self.forward_links[index];
            let Some((old_input, new_input)) = input_change(link.weight()) else {
                continue;
            };
            self.nodes.update_input(link, old_input, new_input);
            self.update_node(link.node());
        }
    }
}

impl JITBackend for DirectBackend {
    fn inspect(&mut self, pos: BlockPos) {
        let Some(node_id) = self.pos_map.get(&pos) else {
            debug!("could not find node at pos {}", pos);
            return;
        };

        debug!("Node {:?}: {:#?}", node_id, self.nodes[*node_id]);
    }

    fn reset<W: World>(&mut self, world: &mut W) {
        self.flush_events(world);
        for (node_id, node) in self.nodes.iter() {
            if node.changed {
                write_blocks(world, &mut self.blocks[node_id.index()], node);
            }
        }
        for (node_id, delay, priority) in self.scheduler.pending() {
            let blocks = &self.blocks[node_id.index()];
            if blocks.is_empty() {
                warn!(
                    "Cannot schedule tick for node {:?} because block information is missing",
                    node_id
                );
            }
            for (pos, _) in blocks.iter().copied() {
                world.schedule_tick(pos, delay, priority);
            }
        }
        self.scheduler = TickScheduler::default();
        self.nodes = Nodes::default();
        self.blocks.clear();
        self.forward_links.clear();
        self.pos_map.clear();
        self.noteblock_info.clear();
        self.events.clear();
    }

    fn on_use_block(&mut self, pos: BlockPos) {
        let node_id = self.pos_map[&pos];
        let node = &self.nodes[node_id];
        match node.ty {
            NodeType::Button => {
                if node.is_powered() {
                    return;
                }
                self.schedule_tick(node_id, 10, TickPriority::Normal);
                self.set_power_and_propagate(node_id, SignalStrength::MAX);
            }
            NodeType::Lever => {
                self.set_power_and_propagate(node_id, (!node.is_powered()).into());
            }
            _ => warn!("Tried to use a {:?} redpiler node", node.ty),
        }
    }

    fn set_pressure_plate(&mut self, pos: BlockPos, powered: bool) {
        let node_id = self.pos_map[&pos];
        let node = &self.nodes[node_id];
        match node.ty {
            NodeType::PressurePlate => {
                self.set_power_and_propagate(node_id, powered.into());
            }
            _ => warn!("Tried to set pressure plate state for a {:?}", node.ty),
        }
    }

    fn tick(&mut self) {
        self.scheduler.advance();
        for priority in scheduler::priorities() {
            let mut due = self.scheduler.take_due(priority);
            while let Some(node_id) = due.next_node(&self.scheduler) {
                self.tick_node(node_id);
            }
        }
    }

    fn flush<W: World>(&mut self, world: &mut W) {
        self.flush_events(world);
        for (node_id, node) in self.nodes.iter_mut() {
            if node.changed && node.visible {
                node.changed = false;
                write_blocks(world, &mut self.blocks[node_id.index()], node);
            }
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

    fn has_pending_ticks(&self) -> bool {
        !self.scheduler.is_empty()
    }
}

fn write_blocks<W: World>(world: &mut W, blocks: &mut [(BlockPos, Block)], node: &Node) {
    for (pos, block) in blocks {
        if let Some(powered) = block_powered_mut(block) {
            *powered = node.is_powered()
        }
        if let Block::IronTrapdoor { open, .. } = block {
            *open = node.is_powered();
        }
        if let Block::RedstoneWire(wire) = block {
            wire.power = node.power.get()
        };
        if let Block::Repeater(repeater) = block {
            repeater.locked = node.locked;
        }
        world.set_block(*pos, *block);
        if matches!(block, Block::Comparator(_)) {
            world.set_block_entity(
                *pos,
                BlockEntity::Comparator {
                    output_strength: node.power.get(),
                },
            );
        }
    }
}

impl fmt::Display for DirectBackend {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "digraph {{")?;
        for (node_id, node) in self.nodes.iter() {
            if matches!(node.ty, NodeType::Wire) {
                continue;
            }
            let id = node_id.index();
            let label = match node.ty {
                NodeType::Gate(gate) => match gate.kind {
                    GateKind::Repeater => format!("Repeater({})", gate.delay),
                    GateKind::Torch => "Torch".to_string(),
                },
                NodeType::Comparator(comparator) => format!(
                    "Comparator({})",
                    match comparator.mode {
                        ComparatorMode::Compare => "Cmp",
                        ComparatorMode::Subtract => "Sub",
                    }
                ),
                NodeType::Lamp => "Lamp".to_string(),
                NodeType::Button => "Button".to_string(),
                NodeType::Lever => "Lever".to_string(),
                NodeType::PressurePlate => "PressurePlate".to_string(),
                NodeType::Trapdoor => "Trapdoor".to_string(),
                NodeType::Wire => "Wire".to_string(),
                NodeType::Constant => format!("Constant({})", node.power),
                NodeType::NoteBlock { .. } => "NoteBlock".to_string(),
            };
            let pos = if !self.blocks[id].is_empty() {
                let mut string = String::new();
                for (idx, (pos, _)) in self.blocks[id].iter().enumerate() {
                    if idx != 0 {
                        write!(&mut string, "; ")?;
                    }
                    write!(&mut string, "{}, {}, {}", pos.x, pos.y, pos.z)?;
                }
                string
            } else {
                "No Pos".to_string()
            };
            writeln!(f, "    n{} [ label = \"{}\\n({})\" ];", id, label, pos)?;
            for index in node.links.iter() {
                let link = self.forward_links[index];
                let out_index = link.node().index();
                let weight = link.weight();
                let color = if link.side() { ",color=\"blue\"" } else { "" };
                writeln!(
                    f,
                    "    n{} -> n{} [ label = \"{}\"{} ];",
                    id, out_index, weight, color
                )?;
            }
        }
        writeln!(f, "}}")
    }
}
