mod compile;
pub mod graph_ops;
mod realtime_channel;

use crate::redpiler::compile_graph::CompileGraph;
use crate::redpiler::{CompilerOptions, TaskMonitor};
use crate::world::World;

use super::JITBackend;
use mchprs_blocks::BlockPos;
use mchprs_world::TickEntry;
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
    }
}
