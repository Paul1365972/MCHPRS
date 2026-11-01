use crate::board::Board;
use crate::build_directory::BuildDirectory;
use crate::engine::FpgaEngine;
use crate::layout::ReadbackScope;
use crate::link::{Controller, SerialLink};
use crate::openxc7::OpenXc7;
use crate::pll::PllSetting;
use crate::progress::Progress;
use crate::toolchain::Toolchain;
use crate::verilog::Design;
use anyhow::Context;
use mchprs_redpiler::backend::GraphContext;
use mchprs_redpiler::circuit::Circuit;
use mchprs_simulation::backend::CompileError;
use std::path::Path;
use tracing::info;

const STAGES: usize = 9;

/// Builds the circuit for the board, programs the board, sets its clock and connects to it.
pub fn run_on_board(
    circuit: &Circuit,
    board: &Board,
    port: Option<&Path>,
    clock_hertz: Option<f64>,
    scope: ReadbackScope,
    build_nonce: u32,
    context: &mut GraphContext<'_>,
) -> Result<FpgaEngine, CompileError> {
    let directory = BuildDirectory::new().context("could not create the build directory");
    let mut directory = directory.map_err(CompileError::Failed)?;
    let build = BoardBuild {
        circuit,
        board,
        port,
        clock_hertz,
        scope,
        build_nonce,
    };
    let engine = build.run(&directory, context);
    engine.map_err(|error| directory.keep_if_a_tool_failed(error))
}

struct BoardBuild<'a> {
    circuit: &'a Circuit,
    board: &'a Board,
    port: Option<&'a Path>,
    clock_hertz: Option<f64>,
    scope: ReadbackScope,
    build_nonce: u32,
}

impl BoardBuild<'_> {
    fn run(
        &self,
        directory: &BuildDirectory,
        context: &mut GraphContext<'_>,
    ) -> Result<FpgaEngine, CompileError> {
        let board = self.board;
        let given_setting = self.clock_hertz.map(|hertz| fastest_setting(board, hertz));
        let given_setting = given_setting.transpose()?;
        let toolchain = OpenXc7;
        let mut progress = Progress::new(context, STAGES);

        progress.start("Lowering")?;
        let design = Design::new(self.circuit, self.scope, self.build_nonce)?;
        progress.start("Writing design")?;
        let sources = board
            .write(&directory.path, &design.source)
            .map_err(failed)?;

        progress.start("Finding board")?;
        let port = board.port(self.port).map_err(failed)?;
        let link = SerialLink::open(&port.path, board.baud)
            .with_context(|| format!("could not open {} for this plot alone", port.path.display()));
        let mut link = link.map_err(CompileError::Failed)?;
        toolchain.detect(board, &port, &directory.path, &progress)?;

        let bitstream = toolchain.build(&sources, board, &mut progress)?;
        info!(
            "{} nodes take {} LUTs and {} flip-flops in {} slices, of which the {} has {}",
            self.circuit.len(),
            bitstream.luts,
            bitstream.flip_flops,
            bitstream.slices,
            board.part,
            board.slices
        );

        progress.start("Programming")?;
        toolchain.program(&bitstream, board, &port, &progress)?;
        link.discard_received().map_err(failed)?;

        progress.start("Setting clock")?;
        let setting = match given_setting {
            Some(setting) => setting,
            None => fastest_setting(board, bitstream.fmax_hertz)?,
        };
        let clock_hertz = setting.output_hertz(board.oscillator_hertz);
        info!(
            "The circuit reaches {:.3} MHz and is clocked at {:.3} MHz",
            bitstream.fmax_hertz / 1e6,
            clock_hertz / 1e6
        );
        let mut controller = Controller::new(Box::new(link));
        set_clock(&mut controller, setting).map_err(CompileError::Failed)?;

        progress.start("Checking board")?;
        let engine = FpgaEngine::connect(controller, self.circuit, &design, clock_hertz);
        let engine = engine.map_err(CompileError::Failed)?;
        progress.finish();
        Ok(engine)
    }
}

fn fastest_setting(board: &Board, hertz: f64) -> Result<PllSetting, CompileError> {
    PllSetting::fastest(board.oscillator_hertz, &board.pll, hertz).ok_or_else(|| {
        let megahertz = hertz / 1e6;
        CompileError::Unsupported(format!(
            "the PLL of the board has no setting as slow as {megahertz:.3} MHz"
        ))
    })
}

/// Writes the PLL's registers for the setting and keeps the bits the setting does not own.
fn set_clock(controller: &mut Controller, setting: PllSetting) -> anyhow::Result<()> {
    for register in setting.registers() {
        let kept = controller.read_pll(register.address)? & register.kept_mask;
        controller.write_pll(register.address, kept | register.value)?;
    }
    Ok(())
}

fn failed(error: impl Into<anyhow::Error>) -> CompileError {
    CompileError::Failed(error.into())
}
