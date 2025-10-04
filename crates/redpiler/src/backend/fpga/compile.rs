use super::node::Node;
use super::FPGABackend;
use crate::compile_graph::{CompileGraph, NodeIdx};
use crate::{CompilerOptions, TaskMonitor};
use mchprs_blocks::blocks::Instrument;
use mchprs_blocks::BlockPos;
use mchprs_world::TickEntry;
use rustc_hash::FxHashMap;
use std::sync::Arc;

#[derive(Debug, Default)]
struct FinalGraphStats {
    update_link_count: usize,
    side_link_count: usize,
    default_link_count: usize,
    nodes_bytes: usize,
}

fn compile_node(
    graph: &CompileGraph,
    node_idx: NodeIdx,
    nodes_len: usize,
    nodes_map: &FxHashMap<NodeIdx, usize>,
    noteblock_info: &mut Vec<(BlockPos, Instrument, u32)>,
    stats: &mut FinalGraphStats,
) -> Node {
    todo!()
}

pub fn compile(
    backend: &mut FPGABackend,
    graph: CompileGraph,
    ticks: Vec<TickEntry>,
    options: &CompilerOptions,
    _monitor: Arc<TaskMonitor>,
) {
    todo!()
}
