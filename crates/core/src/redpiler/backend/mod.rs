pub mod direct;
pub mod parallel;

use std::num::{NonZeroUsize, NonZeroU8};

use super::compile_graph::CompileGraph;
use crate::world::World;
use enum_dispatch::enum_dispatch;
use mchprs_blocks::BlockPos;
use mchprs_world::TickEntry;

#[enum_dispatch]
pub trait JITBackend {
    fn compile(&mut self, graph: CompileGraph, ticks: Vec<TickEntry>);
    fn tick(&mut self);
    fn on_use_block(&mut self, pos: BlockPos);
    fn set_pressure_plate(&mut self, pos: BlockPos, powered: bool);
    fn flush<W: World>(&mut self, world: &mut W, io_only: bool);
    fn reset<W: World>(&mut self, world: &mut W, io_only: bool);
    /// Inspect block for debugging
    fn inspect(&mut self, pos: BlockPos) -> Option<(bool, u8)>;
}

#[cfg(feature = "jit_cranelift")]
use cranelift::CraneliftBackend;
use direct::DirectBackend;
use parallel::ParallelBackend;

#[enum_dispatch(JITBackend)]
pub enum BackendDispatcher {
    DirectBackend,
    ParallelBackend,
    #[cfg(feature = "jit_cranelift")]
    CraneliftBackend,
}

impl Default for BackendDispatcher {
    fn default() -> Self {
        Self::DirectBackend(Default::default())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct NonMaxU8(NonZeroU8);

impl NonMaxU8 {
    fn new(n: u8) -> Option<Self> {
        Some(Self(NonZeroU8::new(n.wrapping_add(1))?))
    }

    fn get(self) -> u8 {
        self.0.get() - 1
    }
}