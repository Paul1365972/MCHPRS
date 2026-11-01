use crate::backend::direct::node::ForwardLinks;
use crate::graph::{Direction, Graph, LinkKind, NodeIdx};
use crate::{CompilerOptions, TaskMonitor};
use mchprs_blocks::blocks::Block;
use rustc_hash::FxHashMap;
use std::sync::Arc;
use tracing::trace;

use super::node::{ForwardLink, Node, NodeId, NodeInput, NodeKind, Nodes};
use super::{DirectBackend, NoteBlockInfo};

#[derive(Debug, Default)]
struct FinalGraphStats {
    update_link_count: usize,
    side_link_count: usize,
    default_link_count: usize,
    nodes_bytes: usize,
}

fn compile_node(
    graph: &Graph,
    node_idx: NodeIdx,
    io_only: bool,
    nodes_map: &FxHashMap<NodeIdx, usize>,
    noteblock_info: &mut Vec<NoteBlockInfo>,
    forward_links: &mut ForwardLinks,
    stats: &mut FinalGraphStats,
) -> Node {
    let node = &graph[node_idx];

    let input_powers = |kind| {
        graph
            .edges(node_idx, Direction::Incoming)
            .filter(move |edge| edge.weight().kind == kind)
            .map(|edge| {
                let link = edge.weight();
                graph[edge.source()].state.power.saturating_sub(link.weight)
            })
    };
    let default_inputs = input_powers(LinkKind::Default)
        .inspect(|_| stats.default_link_count += 1)
        .collect::<NodeInput>();
    let side_inputs = input_powers(LinkKind::Side)
        .inspect(|_| stats.side_link_count += 1)
        .collect::<NodeInput>();

    use crate::graph::NodeKind as GraphNodeKind;
    let fwd_link_range = if node.kind != GraphNodeKind::Constant {
        let new_links = graph
            .edges(node_idx, Direction::Outgoing)
            .map(|edge| unsafe {
                let idx = edge.target();
                let idx = nodes_map[&idx];
                assert!(idx < nodes_map.len());
                // Safety: bounds checked
                let target_id = NodeId::from_index(idx);

                let link = edge.weight();
                ForwardLink::new(target_id, link.kind == LinkKind::Side, link.weight)
            });
        forward_links.extend(new_links)
    } else {
        Default::default()
    };
    stats.update_link_count += fwd_link_range.len();

    let kind = match &node.kind {
        GraphNodeKind::Repeater { delay, prioritized } => NodeKind::Repeater {
            delay: *delay,
            prioritized: *prioritized,
        },
        GraphNodeKind::Torch => NodeKind::Torch,
        GraphNodeKind::Comparator {
            mode,
            far_input,
            prioritized,
        } => NodeKind::Comparator {
            mode: *mode,
            far_input: *far_input,
            prioritized: *prioritized,
        },
        GraphNodeKind::Lamp => NodeKind::Lamp,
        GraphNodeKind::Button => NodeKind::Button,
        GraphNodeKind::Lever => NodeKind::Lever,
        GraphNodeKind::PressurePlate => NodeKind::PressurePlate,
        GraphNodeKind::Trapdoor => NodeKind::Trapdoor,
        GraphNodeKind::Wire => NodeKind::Wire,
        GraphNodeKind::Constant => NodeKind::Constant,
        GraphNodeKind::NoteBlock { instrument, note } => {
            let noteblock_id = noteblock_info.len().try_into().unwrap();
            noteblock_info.push(NoteBlockInfo {
                positions: node.block.iter().copied().map(|(pos, _)| pos).collect(),
                instrument: *instrument,
                note: *note,
                pending: false,
            });
            NodeKind::NoteBlock { noteblock_id }
        }
    };

    Node {
        kind,
        default_inputs,
        side_inputs,
        fwd_link_range,
        power: node.state.power,
        repeater_locked: node.state.repeater_locked,
        changed: false,
        pending_tick: false,
        visible: !io_only || node.port.is_some(),
    }
}

pub fn compile(
    backend: &mut DirectBackend,
    graph: Graph,
    options: &CompilerOptions,
    _monitor: Arc<TaskMonitor>,
) {
    let mut nodes_map = FxHashMap::with_capacity_and_hasher(graph.node_count(), Default::default());
    for node_idx in graph.node_indices() {
        nodes_map.insert(node_idx, nodes_map.len());
    }
    let nodes_len = nodes_map.len();

    // Lower nodes
    let mut stats = FinalGraphStats::default();
    let nodes = graph
        .node_indices()
        .map(|idx| {
            compile_node(
                &graph,
                idx,
                options.io_only,
                &nodes_map,
                &mut backend.noteblock_info,
                &mut backend.forward_links,
                &mut stats,
            )
        })
        .collect();
    stats.nodes_bytes = nodes_len * std::mem::size_of::<Node>();
    trace!("{:#?}", stats);

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
    backend.nodes = Nodes::new(nodes);

    // Create a mapping from block pos to backend NodeId
    for i in 0..backend.blocks.len() {
        for (pos, _) in backend.blocks[i].iter().copied() {
            backend.pos_map.insert(pos, backend.nodes.get(i));
        }
    }

    let mut queued: Vec<_> = graph
        .all_node_weights()
        .enumerate()
        .filter_map(|(index, node)| Some((index, node.state.pending_tick?)))
        .collect();
    queued.sort_by_key(|(_, queued)| queued.order);
    for (index, queued) in queued {
        let node = backend.nodes.get(index);
        backend
            .scheduler
            .schedule_tick(node, queued.tick.delay as usize, queued.tick.priority);
        backend.nodes[node].pending_tick = true;
    }

    // Dot file output
    if options.export_dot_graph {
        std::fs::write("backend_graph.dot", format!("{}", backend)).unwrap();
    }
}
