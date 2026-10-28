use crate::engine::FpgaEngine;
use crate::link::Controller;
use crate::progress::Progress;
use crate::verilator::{self, VerilatorExecutable};
use crate::verilog::Design;
use mchprs_redpiler::backend::{GraphBackend, GraphContext};
use mchprs_redpiler::circuit::Circuit;
use mchprs_redpiler::extract::ExtractOptions;
use mchprs_redpiler::graph_simulation;
use mchprs_redpiler::nodes::Nodes;
use mchprs_redpiler::passes::Pass;
use mchprs_redpiler::ports::WireOutputs;
use mchprs_redpiler_phase::backend::{PhaseBackend, OPTIMIZE};
use mchprs_simulation::backend::{Backend, CompileContext, CompileError, Source};
use mchprs_simulation::layout::Layout;
use mchprs_simulation::{Blocks, Simulation};
use mchprs_syntax::flag::{Flag, FlagValues};

#[derive(Debug, Clone, Copy, Default)]
pub struct FpgaBackend;

impl Backend for FpgaBackend {
    fn name(&self) -> &str {
        "fpga"
    }

    fn flags(&self) -> &[Flag] {
        &[
            OPTIMIZE,
            WireOutputs::DOTS,
            WireOutputs::ILLEGAL_STATES,
            WireOutputs::CROSSES,
        ]
    }

    fn compile(
        &self,
        source: &Source<'_>,
        flags: &FlagValues,
        context: &mut CompileContext<'_>,
    ) -> Result<(Box<dyn Simulation<Blocks>>, Layout), CompileError> {
        graph_simulation::compile(self, source, flags, context)
    }
}

impl GraphBackend for FpgaBackend {
    fn extract_options(&self, flags: &FlagValues) -> ExtractOptions {
        PhaseBackend.extract_options(flags)
    }

    fn passes(&self, flags: &FlagValues) -> Vec<Box<dyn Pass>> {
        PhaseBackend.passes(flags)
    }

    fn build(
        &self,
        circuit: &Circuit,
        _: &FlagValues,
        context: &mut GraphContext<'_>,
    ) -> Result<Box<dyn Simulation<Nodes>>, CompileError> {
        Ok(Box::new(simulate(circuit, context)?))
    }
}

fn simulate(circuit: &Circuit, context: &mut GraphContext<'_>) -> Result<FpgaEngine, CompileError> {
    let mut progress = Progress::new(context, 3);
    progress.start("Writing circuit")?;
    let design = Design::new(circuit, rand::random())?;
    progress.start("Building simulation")?;
    let executable = VerilatorExecutable::build(&design.source, &progress)?;
    let link = executable.start().map_err(failed)?;
    progress.start("Checking circuit")?;
    let controller = Controller::new(Box::new(link));
    let engine = FpgaEngine::connect(controller, circuit, &design, verilator::CIRCUIT_CLOCK_HERTZ);
    let engine = engine.map_err(CompileError::Failed)?;
    progress.finish();
    Ok(engine)
}

fn failed(error: impl Into<anyhow::Error>) -> CompileError {
    CompileError::Failed(error.into())
}
