use super::ThreadedBackend;
use crate::compile_graph::CompileGraph;
use crate::{CompilerOptions, TaskMonitor};
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
