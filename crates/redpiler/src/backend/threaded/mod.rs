mod compile;

use super::JITBackend;
use crate::compile_graph::CompileGraph;
use crate::{CompilerOptions, TaskMonitor};
use mchprs_blocks::BlockPos;
use mchprs_world::{TickEntry, World};
use std::sync::Arc;

#[derive(Default)]
pub struct ThreadedBackend {}

impl ThreadedBackend {}

impl JITBackend for ThreadedBackend {
    fn inspect(&mut self, pos: BlockPos) {}

    fn reset<W: World>(&mut self, world: &mut W, io_only: bool) {}

    fn on_use_block(&mut self, pos: BlockPos) {}

    fn set_pressure_plate(&mut self, pos: BlockPos, powered: bool) {}

    fn tick(&mut self) {}

    fn flush<W: World>(&mut self, world: &mut W, io_only: bool) {}

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
        false
    }
}
