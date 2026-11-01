#![warn(unreachable_pub)]

mod board;
mod cache;
mod circuit;
mod engine;
#[cfg(test)]
mod random_circuits;
mod readback;
mod reference;
mod registers;
mod rtl;
mod stage;
mod subprocess;
mod verilator;

use board::Board;
use circuit::Circuit;
use engine::PhaseEngine;
use mchprs_redpiler::{
    BlockMap, CompileProgress, Engine, EngineError, Netlist, PassStatus, PendingTick, Worker,
};
use mchprs_world::TickEntry;
use reference::Reference;
use registers::Registers;
use stage::Stage;
use std::fmt;
use std::path::Path;
use std::str::FromStr;
use verilator::Verilator;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Target {
    Phase,
    Verilator,
    Fpga,
}

impl Target {
    fn name(self) -> &'static str {
        match self {
            Target::Phase => "phase",
            Target::Verilator => "verilator",
            Target::Fpga => "fpga",
        }
    }

    fn stages(self) -> &'static [Stage] {
        match self {
            Target::Phase => &[Stage::LowerCircuit],
            Target::Verilator => &[
                Stage::LowerCircuit,
                Stage::WriteVerilatorCircuit,
                Stage::GenerateVerilatorModel,
                Stage::CompileVerilatorModel,
                Stage::StartVerilatorCircuit,
            ],
            Target::Fpga => &[
                Stage::LowerCircuit,
                Stage::GenerateVerilog,
                Stage::OpenBoardConnection,
                Stage::SynthesizeLogic,
                Stage::WaitForDeviceDatabase,
                Stage::BuildDeviceDatabase,
                Stage::AssembleDeviceDatabase,
                Stage::PlaceAndRoute,
                Stage::GenerateConfigurationFrames,
                Stage::WriteBitstream,
                Stage::ProgramBoard,
                Stage::VerifyCircuit,
            ],
        }
    }
}

impl fmt::Display for Target {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.name())
    }
}

#[derive(Debug, thiserror::Error)]
#[error("unknown target {0}; use phase, verilator or fpga")]
pub struct UnknownTarget(String);

impl FromStr for Target {
    type Err = UnknownTarget;

    fn from_str(name: &str) -> Result<Self, UnknownTarget> {
        [Target::Phase, Target::Verilator, Target::Fpga]
            .into_iter()
            .find(|target| target.name() == name)
            .ok_or_else(|| UnknownTarget(name.to_owned()))
    }
}

pub fn write_verilog(
    netlist: &Netlist,
    pending_ticks: &[PendingTick],
    directory: &Path,
) -> Result<(), EngineError> {
    let circuit = Circuit::lower(netlist, pending_ticks)?;
    rtl::write(&Registers::new(&circuit), directory)?;
    Ok(())
}

pub fn prepare(
    target: Target,
    netlist: &Netlist,
    pending_ticks: &[PendingTick],
    io_only: bool,
    progress: &CompileProgress,
) -> Result<Box<dyn Engine>, EngineError> {
    let stages = target.stages();
    let report = |stage: Stage| {
        let index = stages
            .iter()
            .position(|&listed| listed == stage)
            .expect("each target lists the stages it reports");
        progress.set(Some(PassStatus {
            index,
            count: stages.len(),
            name: stage.name(),
        }));
    };
    report(Stage::LowerCircuit);
    let circuit = Circuit::lower(netlist, pending_ticks)?;
    let engine: Box<dyn Engine> = match target {
        Target::Phase => Box::new(PhaseEngine::new(
            Reference::new(&circuit),
            &circuit,
            io_only,
        )),
        Target::Verilator => Box::new(PhaseEngine::new(
            Verilator::compile(&circuit, &report)?,
            &circuit,
            io_only,
        )),
        Target::Fpga => Box::new(PhaseEngine::new(
            Board::program(&circuit, &report)?,
            &circuit,
            io_only,
        )),
    };
    progress.set(None);
    Ok(engine)
}

pub fn start(
    target: Target,
    netlist: &Netlist,
    ticks: &[TickEntry],
    io_only: bool,
    progress: &CompileProgress,
) -> Result<Worker, EngineError> {
    let block_map = BlockMap::new(netlist);
    let pending_ticks = block_map.map_ticks(ticks);
    let engine = prepare(target, netlist, &pending_ticks, io_only, progress)?;
    Ok(Worker::start(block_map, engine))
}
