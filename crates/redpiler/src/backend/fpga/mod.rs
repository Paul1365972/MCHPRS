//! The FPGA backend performs code generation, compiles to verilog and runs the simulation iteractively on an FPGA

mod compile;
mod node;

use super::JITBackend;
use crate::compile_graph::CompileGraph;
use crate::task_monitor::TaskMonitor;
use crate::CompilerOptions;
use mchprs_blocks::blocks::{Block, Instrument};
use mchprs_blocks::BlockPos;
use mchprs_world::TickEntry;
use mchprs_world::World;
use node::{NodeId, Nodes};
use rustc_hash::FxHashMap;
use std::sync::Arc;

#[derive(Default)]
pub struct FPGABackend {
    nodes: Nodes,
    blocks: Vec<Option<(BlockPos, Block)>>,
    pos_map: FxHashMap<BlockPos, NodeId>,
    events: Vec<Event>,
    noteblock_info: Vec<(BlockPos, Instrument, u32)>,
}

impl JITBackend for FPGABackend {
    fn inspect(&mut self, pos: BlockPos) {
        todo!()
    }

    fn reset<W: World>(&mut self, world: &mut W, io_only: bool) {
        todo!()
    }

    fn on_use_block(&mut self, pos: BlockPos) {
        todo!()
    }

    fn set_pressure_plate(&mut self, pos: BlockPos, powered: bool) {
        todo!()
    }

    fn tick(&mut self) {
        todo!()
    }

    fn flush<W: World>(&mut self, world: &mut W, io_only: bool) {
        todo!()
    }

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
        todo!()
    }
}

enum Event {
    NoteBlockPlay { noteblock_id: u16 },
}
