use std::sync::Arc;

use itertools::Itertools;
use mchprs_blocks::{
    blocks::{Block, Instrument},
    BlockPos,
};
use mchprs_world::TickEntry;
use rustc_hash::FxHashMap;
use smallvec::SmallVec;
use tracing::trace;

use super::node::{ForwardLink, ForwardLinks, Node, NodeId, NodeInput, NodeType, Nodes};
use super::DirectBackend;
use crate::compile_graph::{
    CompileGraph, Direction, LinkType, NodeIdx, NodeType as CompileNodeType,
};
use crate::{CompilerOptions, TaskMonitor};

#[derive(Debug, Default)]
struct FinalGraphStats {
    update_link_count: usize,
    side_link_count: usize,
    default_link_count: usize,
    nodes_bytes: usize,
}

fn propagation_order(ty: &CompileNodeType) -> u8 {
    match ty {
        CompileNodeType::Repeater { .. } => 0,
        CompileNodeType::Torch => 1,
        CompileNodeType::Comparator { .. } => 2,
        CompileNodeType::Lamp => 3,
        CompileNodeType::Trapdoor => 4,
        CompileNodeType::Wire => 5,
        CompileNodeType::NoteBlock { .. } => 6,
        CompileNodeType::Button
        | CompileNodeType::Lever
        | CompileNodeType::PressurePlate
        | CompileNodeType::Constant => 7,
    }
}

struct Lowering<'a> {
    graph: &'a CompileGraph,
    io_only: bool,
    nodes_map: FxHashMap<NodeIdx, usize>,
    forward_links: ForwardLinks,
    noteblock_info: Vec<(SmallVec<[BlockPos; 1]>, Instrument, u8)>,
    stats: FinalGraphStats,
}

impl<'a> Lowering<'a> {
    fn new(graph: &'a CompileGraph, io_only: bool) -> Self {
        let mut nodes_map =
            FxHashMap::with_capacity_and_hasher(graph.node_count(), Default::default());
        for node_idx in graph.node_indices() {
            nodes_map.insert(node_idx, nodes_map.len());
        }
        Self {
            graph,
            io_only,
            nodes_map,
            forward_links: ForwardLinks::default(),
            noteblock_info: Vec::new(),
            stats: FinalGraphStats::default(),
        }
    }

    fn node(&mut self, node_idx: NodeIdx) -> Node {
        let graph = self.graph;
        let node = &graph[node_idx];

        let input_powers = |ty| {
            graph
                .edges(node_idx, Direction::Incoming)
                .filter(move |edge| edge.weight().ty == ty)
                .map(|edge| {
                    let link = edge.weight();
                    graph[edge.source()].state.power.saturating_sub(link.weight)
                })
        };
        let default_inputs = input_powers(LinkType::Default)
            .inspect(|_| self.stats.default_link_count += 1)
            .collect::<NodeInput>();
        let side_inputs = input_powers(LinkType::Side)
            .inspect(|_| self.stats.side_link_count += 1)
            .collect::<NodeInput>();

        let links = if node.ty != CompileNodeType::Constant {
            let nodes_map = &self.nodes_map;
            let new_links = graph
                .edges(node_idx, Direction::Outgoing)
                .sorted_by_key(|edge| {
                    let target = edge.target();
                    (propagation_order(&graph[target].ty), nodes_map[&target])
                })
                .map(|edge| unsafe {
                    let idx = edge.target();
                    let idx = nodes_map[&idx];
                    assert!(idx < nodes_map.len());
                    // Safety: bounds checked
                    let target_id = NodeId::from_index(idx);

                    let link = edge.weight();
                    ForwardLink::new(target_id, link.ty == LinkType::Side, link.weight)
                });
            self.forward_links.extend(new_links)
        } else {
            Default::default()
        };
        self.stats.update_link_count += links.len();

        let ty = match &node.ty {
            CompileNodeType::Repeater {
                delay,
                facing_diode,
            } => NodeType::Repeater {
                delay: *delay,
                facing_diode: *facing_diode,
            },
            CompileNodeType::Torch => NodeType::Torch,
            CompileNodeType::Comparator {
                mode,
                far_input,
                facing_diode,
            } => NodeType::Comparator {
                mode: *mode,
                far_input: *far_input,
                facing_diode: *facing_diode,
            },
            CompileNodeType::Lamp => NodeType::Lamp,
            CompileNodeType::Button => NodeType::Button,
            CompileNodeType::Lever => NodeType::Lever,
            CompileNodeType::PressurePlate => NodeType::PressurePlate,
            CompileNodeType::Trapdoor => NodeType::Trapdoor,
            CompileNodeType::Wire => NodeType::Wire,
            CompileNodeType::Constant => NodeType::Constant,
            CompileNodeType::NoteBlock { instrument, note } => {
                let noteblock_id = self.noteblock_info.len().try_into().unwrap();
                self.noteblock_info.push((
                    node.block.iter().copied().map(|(pos, _)| pos).collect(),
                    *instrument,
                    *note,
                ));
                NodeType::NoteBlock { noteblock_id }
            }
        };

        Node {
            ty,
            default_inputs,
            side_inputs,
            links,
            power: node.state.power,
            repeater_locked: node.state.repeater_locked,
            changed: false,
            pending_tick: false,
            visible: !self.io_only || node.is_input || node.is_output,
        }
    }
}

pub fn compile(
    backend: &mut DirectBackend,
    graph: CompileGraph,
    ticks: Vec<TickEntry>,
    options: &CompilerOptions,
    _monitor: Arc<TaskMonitor>,
) {
    let mut lowering = Lowering::new(&graph, options.io_only);
    let nodes: Box<[Node]> = graph
        .node_indices()
        .map(|node_idx| lowering.node(node_idx))
        .collect();
    lowering.stats.nodes_bytes = nodes.len() * std::mem::size_of::<Node>();
    trace!("{:#?}", lowering.stats);

    backend.nodes = Nodes::new(nodes);
    backend.forward_links = lowering.forward_links;
    backend.noteblock_info = lowering.noteblock_info;
    backend.blocks = graph
        .all_node_weights()
        .map(|node| {
            node.block
                .iter()
                .copied()
                .map(|(pos, id)| (pos, Block::from_id(id)))
                .collect()
        })
        .collect();

    for (index, blocks) in backend.blocks.iter().enumerate() {
        for (pos, _) in blocks.iter().copied() {
            backend.pos_map.insert(pos, backend.nodes.get(index));
        }
    }

    for entry in ticks {
        if let Some(node_id) = backend.pos_map.get(&entry.pos).copied() {
            backend.schedule_tick(node_id, entry.ticks_left as usize, entry.tick_priority);
        }
    }

    if options.export_dot_graph {
        std::fs::write("backend_graph.dot", format!("{}", backend)).unwrap();
    }
}
