pub mod block_map;
mod running;

use crate::backend::{GraphBackend, GraphContext};
use crate::circuit::{Circuit, InvalidCircuit};
use crate::extract;
use crate::graph::Graph;
use crate::nodes::{Changes, Nodes};
use block_map::BlockMap;
use mchprs_blocks::BlockPos;
use mchprs_redstone::input::{Input, InputError};
use mchprs_simulation::backend::{CompileContext, CompileError, CompileStep, Source, STALE};
use mchprs_simulation::block_changes::BlockChanges;
use mchprs_simulation::input::DriveError;
use mchprs_simulation::layout::Layout;
use mchprs_simulation::readback::Readback;
use mchprs_simulation::ticks::{Ran, SimulationError};
use mchprs_simulation::watch::Watch;
use mchprs_simulation::write_back::WriteBack;
use mchprs_simulation::{Blocks, Capabilities, End, Running, Simulation, View};
use mchprs_syntax::flag::FlagValues;
use running::GraphRunning;

pub fn compile(
    backend: &dyn GraphBackend,
    source: &Source<'_>,
    flags: &FlagValues,
    context: &mut CompileContext<'_>,
) -> Result<(Box<dyn Simulation<Blocks>>, Layout), CompileError> {
    let context = &mut GraphContext::new(context.reborrow());
    let mut graph = extract(backend, source, flags, context)?;
    backend.optimize(&mut graph, flags, context)?;
    build(backend, graph, source, flags, context)
}

pub fn extract(
    backend: &dyn GraphBackend,
    source: &Source<'_>,
    flags: &FlagValues,
    context: &mut GraphContext<'_>,
) -> Result<Graph, CompileError> {
    context.report(CompileStep::single("Extracting components"))?;
    let graph = extract::extract(source, &backend.extract_options(flags))?;
    context.inspect("extracted", &graph);
    Circuit::check(&graph).map_err(|invalid| match invalid {
        InvalidCircuit::Stale { node } => {
            let (pos, _) = graph[node].blocks[0];
            CompileError::Stale(format!("the block at {pos} {STALE}"))
        }
        invalid => invalid.into(),
    })?;
    Ok(graph)
}

/// `graph` comes from `source`.
pub fn build(
    backend: &dyn GraphBackend,
    graph: Graph,
    source: &Source<'_>,
    flags: &FlagValues,
    context: &mut GraphContext<'_>,
) -> Result<(Box<dyn Simulation<Blocks>>, Layout), CompileError> {
    let circuit = Circuit::new(graph)?;
    let simulation = backend.build(&circuit, flags, context)?;
    let blocks = BlockMap::new(circuit, source.region);
    let layout = blocks.layout();
    Ok((Box::new(GraphSimulation { simulation, blocks }), layout))
}

/// A circuit's stepped simulation, addressed by the positions of its blocks.
struct GraphSimulation {
    simulation: Box<dyn Simulation<Nodes>>,
    blocks: BlockMap,
}

impl Simulation<Blocks> for GraphSimulation {
    fn capabilities(&self) -> Capabilities {
        self.simulation.capabilities()
    }

    fn ticks(&self) -> u64 {
        self.simulation.ticks()
    }

    fn run(&mut self, ticks: u64) -> Result<Ran, SimulationError> {
        self.simulation.run(ticks)
    }

    fn settle(&mut self, limit: u64) -> Result<Ran, SimulationError> {
        self.simulation.settle(limit)
    }

    fn drive(&mut self, pos: BlockPos, input: Input) -> Result<(), DriveError> {
        let node = self.blocks.node_at(pos).ok_or(InputError::NotAnInput)?;
        self.simulation.drive(node, input)
    }

    fn watch(&mut self, watch: Watch<BlockPos>) -> Result<(), SimulationError> {
        self.simulation.watch(self.blocks.watch(watch))
    }

    fn show(&mut self, view: View, changes: &mut BlockChanges) -> Result<(), SimulationError> {
        let mut node_changes = Changes::default();
        self.simulation.show(view, &mut node_changes)?;
        self.blocks.record(&node_changes, changes);
        Ok(())
    }

    fn read(&mut self) -> Result<Readback<WriteBack>, SimulationError> {
        let state = self.simulation.read()?;
        Ok(state.map(|snapshot| self.blocks.write_back(&snapshot)))
    }

    fn stop(self: Box<Self>) -> Result<End<Blocks>, SimulationError> {
        let GraphSimulation {
            simulation,
            mut blocks,
        } = *self;
        Ok(blocks.end(simulation.stop()?))
    }

    fn start(self: Box<Self>) -> Result<Box<dyn Running<Blocks>>, SimulationError> {
        let GraphSimulation { simulation, blocks } = *self;
        let ticks = simulation.ticks();
        let running = simulation.start()?;
        Ok(Box::new(GraphRunning::new(running, blocks, ticks)))
    }
}
