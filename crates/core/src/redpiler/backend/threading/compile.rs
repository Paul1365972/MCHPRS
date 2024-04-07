use std::cmp::Reverse;
use std::sync::Arc;

use itertools::Itertools;
use mchprs_blocks::blocks::Block;
use mchprs_world::TickEntry;
use petgraph::Direction;
use realtime_channel::RingBuffer;
use rustc_hash::{FxHashMap, FxHashSet};
use smallvec::SmallVec;
use thread_priority::{ThreadBuilder, ThreadPriority};
use tracing::trace;

use crate::redpiler::backend::common::{ForwardLink, NodeIdWithData, NodeInput};
use crate::redpiler::backend::nodes::NodeId;
use crate::redpiler::backend::threading::{ChangeMessage, TickScheduler, UpdateMessage};
use crate::redpiler::compile_graph::{CompileGraph, LinkType, NodeIdx};
use crate::redpiler::{CompilerOptions, TaskMonitor};

use super::node::Node;
use super::*;

#[derive(Debug, Default)]
struct FinalGraphStats {
    update_link_count: usize,
    side_link_count: usize,
    default_link_count: usize,
    nodes_bytes: usize,
}

impl Node {
    fn from_compile_node(
        graph: &CompileGraph,
        node_idx: NodeIdx,
        nodes_map: &FxHashMap<NodeIdx, (WorkerId, NodeId)>,
        output_indicies: &mut Vec<(WorkerId, u32)>,
        stats: &mut FinalGraphStats,
    ) -> Self {
        let node = &graph[node_idx];

        const MAX_INPUTS: usize = 255;

        let mut default_input_count = 0;
        let mut side_input_count = 0;

        let mut default_inputs = NodeInput::default();
        let mut side_inputs = NodeInput::default();
        let mut default_input = 0;
        let mut side_input = 0;
        for edge in graph.edges_directed(node_idx, Direction::Incoming) {
            let weight = edge.weight();
            let distance = weight.ss;
            let source = edge.source();
            let ss = graph[source].state.output_strength.saturating_sub(distance);
            match weight.ty {
                LinkType::Default => {
                    if default_input_count >= MAX_INPUTS {
                        panic!(
                            "Exceeded the maximum number of default inputs {}",
                            MAX_INPUTS
                        );
                    }
                    default_input_count += 1;
                    if node.is_analog() {
                        default_inputs.ss_counts[ss as usize] += 1;
                    } else if ss > 0 {
                        default_input += 1;
                    }
                }
                LinkType::Side => {
                    if side_input_count >= MAX_INPUTS {
                        panic!("Exceeded the maximum number of side inputs {}", MAX_INPUTS);
                    }
                    side_input_count += 1;

                    if node.is_analog() {
                        side_inputs.ss_counts[ss as usize] += 1;
                    } else if ss > 0 {
                        side_input += 1;
                    }
                }
            }
        }
        stats.default_link_count += default_input_count;
        stats.side_link_count += side_input_count;

        use crate::redpiler::compile_graph::NodeType as CNodeType;
        let updates = if node.ty != CNodeType::Constant {
            graph
                .edges_directed(node_idx, Direction::Outgoing)
                .map(|edge| {
                    let idx = edge.target();
                    let (_, node_id) = nodes_map[&idx];

                    let weight = edge.weight();
                    ForwardLink::new(node_id, weight.ty == LinkType::Side, weight.ss)
                })
                .collect()
        } else {
            SmallVec::new()
        };
        stats.update_link_count += updates.len();

        if node.is_analog() {
            default_input = default_inputs.get_ss();
            side_input = side_inputs.get_ss();
        }

        let ty = match &node.ty {
            CNodeType::Repeater {
                delay,
                facing_diode,
            } => NodeType::Repeater {
                delay: *delay,
                facing_diode: *facing_diode,
            },

            CNodeType::Torch => NodeType::Torch,
            CNodeType::Comparator {
                mode,
                far_input,
                facing_diode,
            } => NodeType::Comparator {
                mode: *mode,
                far_input: far_input.map(|x| NonMaxU8::new(x).unwrap()),
                facing_diode: *facing_diode,
            },
            CNodeType::Lamp => NodeType::Lamp,
            CNodeType::Button => NodeType::Button,
            CNodeType::Lever => NodeType::Lever,
            CNodeType::PressurePlate => NodeType::PressurePlate,
            CNodeType::Trapdoor => NodeType::Trapdoor,
            CNodeType::Wire => NodeType::Wire,
            CNodeType::Constant => NodeType::Constant,
            CNodeType::ExternalInput => NodeType::ExternalInput,
            CNodeType::ExternalOutput { target_idx, delay } => {
                let (worker_id, target_id) = nodes_map[target_idx];
                let output_index = output_indicies
                    .iter()
                    .position(|(id, _)| *id == worker_id)
                    .unwrap_or_else(|| {
                        output_indicies.push((worker_id, *delay));
                        output_indicies.len() - 1
                    })
                    .try_into()
                    .unwrap();
                assert_eq!(output_indicies[output_index as usize].1, *delay);
                NodeType::ExternalOutput {
                    output_index,
                    target_id,
                }
            }
            //CNodeType::ComparatorLine { .. } => unreachable!(),
            CNodeType::ComparatorLine { states } => NodeType::ComparatorLine {
                delay: {
                    let delay = states.len();
                    assert!(delay >= 2 && delay < TickScheduler::NUM_QUEUES);
                    delay.try_into().unwrap()
                },
            },
            // TODO: Threading
            CNodeType::NoteBlock { .. } => todo!(),
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
            default_power: default_input,
            side_power: side_input,
        }
    }
}

pub fn compile(
    backend: &mut ThreadingBackend,
    graph: CompileGraph,
    ticks: Vec<TickEntry>,
    _options: &CompilerOptions,
    _monitor: Arc<TaskMonitor>,
) {
    let groups = weakly_connected_components(&graph);
    let big_group_count = groups
        .iter()
        .filter(|group| group.len() > graph.node_count() / 10_000)
        .count();

    trace!(
        "Found groups. Big groups: {}, Small groups: {}",
        big_group_count,
        groups.len()
    );

    trace!(
        "Partitioned groups. Count: {}, Sizes: {:?}",
        groups.len(),
        groups
            .iter()
            .map(|group| group.len())
            .sorted()
            .collect_vec()
    );

    let thread_count = std::thread::available_parallelism().unwrap().get() - 1;
    let mut groups = multiway_number_partitioning(groups, big_group_count.min(thread_count).max(1));
    trace!(
        "Partitioned groups. Count: {}, Sizes: {:?}",
        groups.len(),
        groups.iter().map(|group| group.len()).collect_vec()
    );

    let mut workers = vec![];

    for group in groups.iter_mut() {
        group.sort_by_key(|&idx| Reverse(graph[idx].is_io_and_flushable()));
    }

    let nodes_map: FxHashMap<NodeIdx, (WorkerId, NodeId)> = groups
        .iter()
        .enumerate()
        .flat_map(|(worker_id, group)| {
            group.iter().enumerate().map(move |(node_id, node_idx)| {
                (
                    *node_idx,
                    (WorkerId(worker_id.try_into().unwrap()), unsafe {
                        NodeId::from_index(node_id)
                    }),
                )
            })
        })
        .collect();

    let mut worker_output_indicies = vec![];

    for (worker_id, group) in groups.into_iter().enumerate() {
        let mut stats = FinalGraphStats::default();
        let mut nodes = vec![];
        let mut blocks = vec![];
        let mut output_indicies = vec![];
        let mut last_io_index = 0;
        let mut tickable_nodes = 0;

        for node_idx in group.iter() {
            let (worker_id, node_id) = nodes_map[node_idx];
            let node = &graph[*node_idx];

            if let Some((pos, id)) = node.block {
                blocks.push(Some((pos, Block::from_id(id))));
                backend.pos_map.insert(pos, (worker_id, node_id));
            } else {
                blocks.push(None);
            }

            if node.is_io_and_flushable() {
                last_io_index = last_io_index.max(node_id.index());
            }

            if node.can_be_ticked() {
                tickable_nodes += 1;
            }

            nodes.push(Node::from_compile_node(
                &graph,
                *node_idx,
                &nodes_map,
                &mut output_indicies,
                &mut stats,
            ));
        }
        stats.nodes_bytes = nodes.len() * std::mem::size_of::<Node>();
        trace!("{:#?}", stats);

        let (mut change_sender, change_receiver) = RingBuffer::new(CHANGES_CHANNEL_SIZE);
        for _ in 0..FLUSH_OFFSET {
            change_sender.send(ChangeMessage::ChangesEnd);
        }
        backend.change_receivers.push(change_receiver);

        let (work_sender, work_receiver) = RingBuffer::new(WORK_CHANNEL_SIZE);
        backend.work_senders.push(work_sender);

        let scheduler = unsafe { TickScheduler::new(tickable_nodes) };

        let worker = Worker {
            id: WorkerId(worker_id.try_into().unwrap()),
            inputs: vec![],
            outputs: vec![],
            work_receiver,
            change_sender,
            nodes: Nodes::new(nodes.into_boxed_slice()),
            scheduler,
            blocks,
            last_io_index,
        };
        // Dot file output
        // println!("{}", worker);
        workers.push(worker);
        worker_output_indicies.push(output_indicies);
    }

    for worker_id in 0..workers.len() {
        for &(target_id, delay) in worker_output_indicies[worker_id].iter() {
            assert_ne!(delay, 0);
            let (mut sender, receiver) = RingBuffer::new(UPDATES_CHANNEL_SIZE);
            for _ in 0..(delay as usize * TickScheduler::NUM_PRIORITIES) {
                sender.send(UpdateMessage::UpdatesEnd);
            }
            workers[worker_id].outputs.push(sender);
            workers[target_id.0 as usize].inputs.push(receiver);
        }
    }

    for entry in ticks {
        if let Some((worker_id, node_id)) = backend.pos_map.get(&entry.pos) {
            let worker = &mut workers[worker_id.0 as usize];
            worker.scheduler.schedule_tick(
                NodeIdWithData::new(*node_id),
                entry.ticks_left as usize,
                entry.tick_priority,
            );
            worker.nodes[*node_id].pending_tick = true;
        }
    }

    for worker in workers {
        backend.handles.push(
            ThreadBuilder::default()
                .name(format!("Worker Thread {}", worker.id.0))
                .priority(ThreadPriority::Max)
                .spawn_careless(|| worker.run())
                .expect("failed to spawn worker thread"),
        );
    }
}

pub fn weakly_connected_components(graph: &CompileGraph) -> Vec<Vec<NodeIdx>> {
    let mut visited = FxHashSet::with_capacity_and_hasher(graph.node_count(), Default::default());
    let mut components = vec![];

    for node in graph.node_indices() {
        if !visited.contains(&node) {
            visited.insert(node);

            let mut component = vec![node];
            let mut index = 0;
            while component.len() > index {
                for neighbor in graph.neighbors_undirected(component[index]) {
                    if !visited.contains(&neighbor) {
                        visited.insert(neighbor);
                        component.push(neighbor);
                    }
                }
                index += 1;
            }
            components.push(component);
        }
    }
    components
}

// Merge groups into `k` bins using a greedy algorithm for the multiway number partitioning problem.
// TODO: In the future a higher quality algorithm should be used here.
pub fn multiway_number_partitioning(mut groups: Vec<Vec<NodeIdx>>, k: usize) -> Vec<Vec<NodeIdx>> {
    if groups.len() <= k {
        return groups;
    }
    groups.sort_by_key(|group| Reverse(group.len()));

    let mut bins: BinaryHeap<(Reverse<usize>, Vec<NodeIdx>)> = BinaryHeap::with_capacity(k);
    for _ in 0..k {
        bins.push((Reverse(0), vec![]));
    }

    for group in groups {
        let (_, mut bin) = bins.pop().unwrap();
        bin.extend(group);
        bins.push((Reverse(bin.len()), bin));
    }
    bins.into_iter().map(|(_, bin)| bin).collect()
}
