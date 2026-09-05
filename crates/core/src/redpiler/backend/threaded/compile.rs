use super::ThreadedBackend;
use crate::redpiler::compile_graph::CompileGraph;
use crate::redpiler::{CompilerOptions, TaskMonitor};
use mchprs_world::TickEntry;
use std::sync::Arc;

pub fn compile(
    backend: &mut ThreadedBackend,
    graph: CompileGraph,
    ticks: Vec<TickEntry>,
    options: &CompilerOptions,
    _monitor: Arc<TaskMonitor>,
) {
}
