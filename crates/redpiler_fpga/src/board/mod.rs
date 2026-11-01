mod device;
mod toolchain;

use crate::circuit::{Circuit, State};
use crate::engine::Simulator;
use crate::readback::Readback;
use crate::registers::Registers;
use crate::rtl;
use device::{Command, Device};
use std::io;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};

static BUILD_SEQUENCE: AtomicU64 = AtomicU64::new(0);

pub(crate) struct Board {
    device: Device,
    ticks: u64,
    readback: Readback,
    nodes: Vec<usize>,
}

impl Board {
    pub(crate) fn program(circuit: &Circuit, progress: &dyn Fn(&'static str)) -> io::Result<Board> {
        let port = std::env::var("MCHPRS_FPGA_PORT").map_err(|_| {
            io::Error::other("set MCHPRS_FPGA_PORT to the board UART device before compiling")
        })?;
        let root = std::env::var_os("MCHPRS_FPGA_CACHE")
            .map(PathBuf::from)
            .unwrap_or_else(|| std::env::temp_dir().join("mchprs-fpga"));
        let directory = root.join(format!(
            "{:016x}-{}-{}",
            circuit.identity(),
            std::process::id(),
            BUILD_SEQUENCE.fetch_add(1, Ordering::Relaxed)
        ));
        progress("Generating Verilog");
        let registers = Registers::new(circuit);
        rtl::write(&registers, &directory)?;
        progress("Opening board connection");
        let readback = Readback::new(&registers);
        let nodes: Vec<_> = (0..circuit.nodes().len()).collect();
        let mut device = Device::serial(
            &port,
            circuit,
            readback.bytes(nodes.iter().copied()),
            readback.bytes(circuit.io_nodes().iter().copied()),
        )?;
        let bitstream = toolchain::build(&directory, progress)?;
        progress("Programming board");
        toolchain::program(&bitstream)?;
        progress("Verifying circuit");
        if device.execute(Command::Identity)?.ticks != circuit.identity() {
            return Err(io::Error::other(
                "board circuit identity does not match the compiled circuit",
            ));
        }
        let programmed = device.execute(Command::Snapshot)?;
        let initial: Vec<_> = circuit.nodes().iter().map(|node| node.initial).collect();
        if programmed.ticks != 0
            || readback.decode(nodes.iter().copied(), &programmed.data)? != initial
        {
            return Err(io::Error::other(
                "programmed board state does not match the compiled circuit",
            ));
        }
        if let Err(error) = remove_build_artifacts(&directory) {
            tracing::warn!(%error, directory = %directory.display(), "could not remove the FPGA build artifacts");
        }
        Ok(Board {
            device,
            ticks: 0,
            readback,
            nodes,
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
    fn input(&mut self, node: usize, powered: bool) -> io::Result<()> {
        self.device.execute(Command::Input {
            index: node as u32,
            powered,
        })?;
        Ok(())
    }

    fn advance(&mut self, ticks: u64) -> io::Result<()> {
        let target = self.ticks + ticks;
        let mut reached = self.device.execute(Command::RunUntil(target))?.ticks;
        while reached < target {
            reached = self.device.execute(Command::Status)?.ticks;
        }
        if reached != target {
            return Err(io::Error::other(format!(
                "board ran to tick {reached} instead of {target}"
            )));
        }
        self.ticks = target;
        Ok(())
    }

    fn states(&mut self) -> io::Result<Vec<State>> {
        let data = self.device.execute(Command::Snapshot)?.data;
        self.readback.decode(self.nodes.iter().copied(), &data)
    }

    fn io_states(&mut self, io_nodes: &[usize]) -> io::Result<Vec<State>> {
        let data = self.device.execute(Command::Outputs)?.data;
        self.readback.decode(io_nodes.iter().copied(), &data)
    }
}
