pub mod direct;

use std::sync::Arc;

use super::graph::Graph;
use super::task_monitor::TaskMonitor;
use super::CompilerOptions;
use enum_dispatch::enum_dispatch;
use mchprs_blocks::BlockPos;
use mchprs_redstone::RedstoneWorld;

#[enum_dispatch]
pub trait JITBackend {
    fn compile(&mut self, graph: Graph, options: &CompilerOptions, monitor: Arc<TaskMonitor>);
    fn tick(&mut self);

    fn tickn(&mut self, ticks: u64) {
        for _ in 0..ticks {
            self.tick();
        }
    }

    fn on_use_block(&mut self, pos: BlockPos);
    fn set_pressure_plate(&mut self, pos: BlockPos, powered: bool);
    fn flush<W: RedstoneWorld>(&mut self, world: &mut W);
    fn flush_all<W: RedstoneWorld>(&mut self, world: &mut W);
    fn reset<W: RedstoneWorld>(&mut self, world: &mut W);
    fn has_pending_ticks(&self) -> bool;
    /// Inspect block for debugging
    fn inspect(&mut self, pos: BlockPos);
}

use direct::DirectBackend;

#[enum_dispatch(JITBackend)]
pub enum BackendDispatcher {
    DirectBackend,
}
