mod board;
mod circuit;
mod engine;
#[cfg(test)]
mod random_circuits;
mod readback;
mod reference;
mod registers;
mod rtl;
mod verilator;

use board::Board;
use circuit::Circuit;
use engine::PhaseEngine;
use mchprs_redpiler::{
    BlockMap, CompileProgress, Engine, EngineError, Netlist, PassStatus, PendingTick, Worker,
};
use mchprs_world::TickEntry;
use reference::Reference;
use verilator::VerilatorEngine;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Target {
    Reference,
    Verilator,
    Board,
}

impl Target {
    pub fn from_name(name: &str) -> Option<Target> {
        match name {
            "phase" => Some(Target::Reference),
            "verilator" => Some(Target::Verilator),
            "fpga" => Some(Target::Board),
            _ => None,
        }
    }

    fn stages(self) -> &'static [&'static str] {
        match self {
            Target::Reference => &["Lowering circuit"],
            Target::Verilator => &[
                "Lowering circuit",
                "Writing Verilator circuit",
                "Generating Verilator model",
                "Compiling Verilator model",
                "Starting Verilator circuit",
            ],
            Target::Board => &[
                "Lowering circuit",
                "Generating Verilog",
                "Opening board connection",
                "Synthesizing logic",
                "Waiting for device database",
                "Building device database",
                "Assembling device database",
                "Placing and routing",
                "Generating configuration frames",
                "Writing bitstream",
                "Programming board",
                "Verifying circuit",
            ],
        }
    }
}

pub fn write_verilog(
    netlist: &Netlist,
    pending_ticks: &[PendingTick],
    directory: &std::path::Path,
) -> Result<(), EngineError> {
    Circuit::lower(netlist, pending_ticks)?.write(directory)?;
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
    let report = |name: &'static str| {
        let index = stages.iter().position(|stage| *stage == name).unwrap();
        progress.set(Some(PassStatus {
            index,
            count: stages.len(),
            name,
        }));
    };
    report("Lowering circuit");
    let circuit = Circuit::lower(netlist, pending_ticks)?;
    let engine: Box<dyn Engine> = match target {
        Target::Reference => Box::new(PhaseEngine::new(
            Reference::new(&circuit)?,
            &circuit,
            io_only,
        )),
        Target::Verilator => Box::new(PhaseEngine::new(
            VerilatorEngine::compile(&circuit, &report)?,
            &circuit,
            io_only,
        )),
        Target::Board => Box::new(PhaseEngine::new(
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
