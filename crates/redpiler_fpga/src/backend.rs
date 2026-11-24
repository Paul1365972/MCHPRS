use crate::board::BASYS3;
use crate::bring_up;
use crate::engine::{ClockPace, FpgaEngine};
use crate::layout::ReadbackScope;
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
use mchprs_redpiler_phase::backend::{PhaseBackend, IO_ONLY, OPTIMIZE};
use mchprs_simulation::backend::{Backend, CompileContext, CompileError, Source};
use mchprs_simulation::layout::Layout;
use mchprs_simulation::{Blocks, Simulation};
use mchprs_syntax::flag::{Flag, FlagValues};
use std::path::Path;

const SIMULATE: Flag = Flag::switch(
    "simulate",
    None,
    "Run the circuit in Verilator instead of on the board",
);
const PORT: Flag = Flag::valued(
    "port",
    "path",
    "The serial port of the board, if more than one is attached",
);

#[derive(Debug, Clone, Copy, Default)]
pub struct FpgaBackend;

impl Backend for FpgaBackend {
    fn name(&self) -> &str {
        "fpga"
    }

    fn flags(&self) -> &[Flag] {
        &[
            OPTIMIZE,
            IO_ONLY,
            WireOutputs::DOTS,
            WireOutputs::ILLEGAL_STATES,
            WireOutputs::CROSSES,
            PORT,
            SIMULATE,
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
        flags: &FlagValues,
        context: &mut GraphContext<'_>,
    ) -> Result<Box<dyn Simulation<Nodes>>, CompileError> {
        Ok(Box::new(build_engine(circuit, flags, context)?))
    }
}

fn build_engine(
    circuit: &Circuit,
    flags: &FlagValues,
    context: &mut GraphContext<'_>,
) -> Result<FpgaEngine, CompileError> {
    let build_nonce = rand::random();
    let scope = match flags.is_on(&IO_ONLY) {
        true => ReadbackScope::Observed,
        false => ReadbackScope::All,
    };
    match (flags.is_on(&SIMULATE), flags.text(&PORT)) {
        (true, None) => simulate(circuit, scope, build_nonce, context),
        (true, Some(_)) => Err(CompileError::Unsupported(format!(
            "a simulation has no --{}",
            PORT.name()
        ))),
        (false, port) => {
            let port = port.map(Path::new);
            let board = &BASYS3;
            bring_up::run_on_board(circuit, board, port, scope, build_nonce, context)
        }
    }
}

fn simulate(
    circuit: &Circuit,
    scope: ReadbackScope,
    build_nonce: u32,
    context: &mut GraphContext<'_>,
) -> Result<FpgaEngine, CompileError> {
    let mut progress = Progress::new(context, 4);
    progress.start("Writing circuit")?;
    let design = Design::new(circuit, scope, build_nonce)?;
    let executable = VerilatorExecutable::build(&design.source, &mut progress)?;
    let link = executable.start();
    let link = link.map_err(|error| CompileError::Failed(error.into()))?;
    progress.start("Checking circuit")?;
    let controller = Controller::new(Box::new(link));
    let clock_hertz = verilator::CIRCUIT_CLOCK_HERTZ;
    let engine = FpgaEngine::connect(
        controller,
        circuit,
        &design,
        clock_hertz,
        ClockPace::Simulated,
    );
    let engine = engine.map_err(CompileError::Failed)?;
    progress.finish();
    Ok(engine)
}
