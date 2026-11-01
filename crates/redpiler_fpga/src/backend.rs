use crate::board::BASYS3;
use crate::bring_up;
use crate::engine::FpgaEngine;
use crate::layout::ReadbackScope;
use crate::link::Controller;
use crate::progress::Progress;
use crate::verilator::{Timing, VerilatorExecutable};
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
use std::path::Path;

const IO_ONLY: Flag = Flag::switch(
    "io-only",
    Some('i'),
    "Only update inputs, outputs and probed components, and only read their state from the board",
);
const SIMULATE: Flag = Flag::switch(
    "simulate",
    None,
    "Run the circuit in Verilator instead of on the board",
);
const PORT: Flag = Flag::valued(
    "port",
    "path",
    "The serial port of the board, where several boards are attached",
);
const CLOCK: Flag = Flag::valued(
    "clock",
    "MHz",
    "The fastest clock for the circuit, instead of what the routed circuit is timed to reach",
);

const SIMULATED_TIMING: Timing = Timing {
    board_half_period: 3,
    circuit_half_period: 2,
    clocks_per_byte: 1,
};

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
            CLOCK,
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
    let board_flag = [PORT, CLOCK].into_iter().find(|flag| flags.contains(flag));
    match (flags.is_on(&SIMULATE), board_flag) {
        (true, None) => simulate(circuit, scope, build_nonce, context),
        (true, Some(flag)) => Err(CompileError::Unsupported(format!(
            "a simulation has no --{}",
            flag.name()
        ))),
        (false, _) => {
            let port = flags.text(&PORT).map(Path::new);
            let clock_hertz = given_clock_hertz(flags)?;
            let board = &BASYS3;
            bring_up::run_on_board(
                circuit,
                board,
                port,
                clock_hertz,
                scope,
                build_nonce,
                context,
            )
        }
    }
}

fn given_clock_hertz(flags: &FlagValues) -> Result<Option<f64>, CompileError> {
    let Some(text) = flags.text(&CLOCK) else {
        return Ok(None);
    };
    let megahertz: f64 = text.parse().map_err(|error| {
        CompileError::Unsupported(format!(
            "--{} takes a frequency in MHz: {error}",
            CLOCK.name()
        ))
    })?;
    Ok(Some(megahertz * 1e6))
}

fn simulate(
    circuit: &Circuit,
    scope: ReadbackScope,
    build_nonce: u32,
    context: &mut GraphContext<'_>,
) -> Result<FpgaEngine, CompileError> {
    let mut progress = Progress::new(context, 3);
    progress.start("Writing circuit")?;
    let design = Design::new(circuit, scope, build_nonce)?;
    progress.start("Building simulation")?;
    let executable = VerilatorExecutable::build(&design.source, &progress)?;
    let link = executable.start(SIMULATED_TIMING);
    let link = link.map_err(|error| CompileError::Failed(error.into()))?;
    progress.start("Checking circuit")?;
    let clock_hertz = SIMULATED_TIMING.circuit_clock_hertz();
    let controller = Controller::new(Box::new(link));
    let engine = FpgaEngine::connect(controller, circuit, &design, clock_hertz);
    let engine = engine.map_err(CompileError::Failed)?;
    progress.finish();
    Ok(engine)
}
