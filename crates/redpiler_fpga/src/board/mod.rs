mod device;
mod toolchain;

use crate::cache;
use crate::circuit::{Circuit, State};
use crate::engine::Simulator;
use crate::readback::{Readback, ReadbackError};
use crate::registers::Registers;
use crate::rtl;
use crate::stage::Stage;
use device::{Command, Device, DeviceError};
use mchprs_redpiler::Reporting;
use std::io;
use std::path::Path;
use toolchain::ToolchainError;

const PORT_VARIABLE: &str = "MCHPRS_FPGA_PORT";

#[derive(Debug, thiserror::Error)]
pub(crate) enum BoardError {
    #[error("set {PORT_VARIABLE} to the board UART device before compiling")]
    MissingPort,
    #[error("could not write the FPGA circuit")]
    Write(#[source] io::Error),
    #[error(transparent)]
    Device(#[from] DeviceError),
    #[error(transparent)]
    Toolchain(#[from] ToolchainError),
    #[error(transparent)]
    Readback(#[from] ReadbackError),
    #[error("board circuit identity does not match the compiled circuit")]
    Identity,
    #[error("programmed board state does not match the compiled circuit")]
    State,
    #[error("board ran to tick {reached} instead of {target}")]
    Ticks { reached: u64, target: u64 },
    #[error("board build reads back only input and output nodes")]
    NoStateReadback,
}

pub(crate) struct Board {
    device: Device,
    ticks: u64,
    readback: Readback,
    node_count: usize,
    snapshots_every_block: bool,
}

impl Board {
    pub(crate) fn program(
        circuit: &Circuit,
        reporting: Reporting,
        progress: &dyn Fn(Stage),
    ) -> Result<Board, BoardError> {
        let port = std::env::var(PORT_VARIABLE).map_err(|_| BoardError::MissingPort)?;
        let directory = cache::unique_build_directory(&cache::board_root(), "", circuit.identity());
        progress(Stage::GenerateVerilog);
        let registers = Registers::new(circuit);
        rtl::write(&registers, reporting, &directory).map_err(BoardError::Write)?;
        progress(Stage::OpenBoardConnection);
        let readback = Readback::new(&registers);
        let node_count = circuit.nodes().len();
        let snapshots_every_block = reporting.snapshots_every_block();
        let (snapshot, snapshot_nodes): (_, Vec<_>) = if snapshots_every_block {
            (Command::Snapshot, (0..node_count).collect())
        } else {
            (Command::Outputs, circuit.io_nodes().to_vec())
        };
        let mut device = Device::serial(
            &port,
            circuit,
            if snapshots_every_block {
                readback.bytes(0..node_count)
            } else {
                0
            },
            readback.bytes(circuit.io_nodes().iter().copied()),
        )?;
        let bitstream = toolchain::build(&directory, progress)?;
        progress(Stage::ProgramBoard);
        toolchain::program(&bitstream)?;
        progress(Stage::VerifyCircuit);
        if device.execute(Command::Identity)?.ticks != circuit.identity() {
            return Err(BoardError::Identity);
        }
        let programmed = device.execute(snapshot)?;
        let initial: Vec<_> = snapshot_nodes
            .iter()
            .map(|&node| circuit.nodes()[node].initial)
            .collect();
        let states = readback.decode(snapshot_nodes.iter().copied(), &programmed.data)?;
        if programmed.ticks != 0 || states != initial {
            return Err(BoardError::State);
        }
        if let Err(error) = remove_build_artifacts(&directory) {
            tracing::warn!(%error, directory = %directory.display(), "could not remove the FPGA build artifacts");
        }
        Ok(Board {
            device,
            ticks: 0,
            readback,
            node_count,
            snapshots_every_block,
        })
    }
}

fn remove_build_artifacts(directory: &Path) -> io::Result<()> {
    for entry in std::fs::read_dir(directory)? {
        let path = entry?.path();
        if path.is_dir() {
            std::fs::remove_dir_all(path)?;
        } else if path.extension().is_none_or(|extension| extension != "log") {
            std::fs::remove_file(path)?;
        }
    }
    Ok(())
}

impl Simulator for Board {
    type Error = BoardError;

    fn input(&mut self, node: usize, powered: bool) -> Result<(), BoardError> {
        self.device.execute(Command::Input {
            index: node as u32,
            powered,
        })?;
        Ok(())
    }

    fn advance(&mut self, ticks: u64) -> Result<(), BoardError> {
        let target = self.ticks + ticks;
        let mut reached = self.device.execute(Command::RunUntil(target))?.ticks;
        while reached < target {
            reached = self.device.execute(Command::Status)?.ticks;
        }
        if reached != target {
            return Err(BoardError::Ticks { reached, target });
        }
        self.ticks = target;
        Ok(())
    }

    fn states(&mut self) -> Result<Vec<State>, BoardError> {
        if !self.snapshots_every_block {
            return Err(BoardError::NoStateReadback);
        }
        let data = self.device.execute(Command::Snapshot)?.data;
        Ok(self.readback.decode(0..self.node_count, &data)?)
    }

    fn io_states(&mut self, io_nodes: &[usize]) -> Result<Vec<State>, BoardError> {
        let data = self.device.execute(Command::Outputs)?.data;
        Ok(self.readback.decode(io_nodes.iter().copied(), &data)?)
    }
}
