mod device;
mod pll;
mod toolchain;

use crate::cache;
use crate::circuit::{Circuit, State};
use crate::engine::Observer;
use crate::readback::{Readback, ReadbackError};
use crate::registers::{Contents, Registers};
use crate::rtl;
use crate::stage::Stage;
use device::{Board, Command, DeviceError};
use mchprs_redpiler::{
    Engine, EngineError, EngineSnapshot, Input, NodeChanges, NodeId, Progress, Reporting, Resume,
};
use mchprs_world::{SendRate, TickRate};
use std::io;
use std::path::Path;
use std::time::{Duration, Instant};
use toolchain::ToolchainError;

const PORT_VARIABLE: &str = "MCHPRS_FPGA_PORT";
/// The frequency of the board's oscillator, which clocks the serial line and the PLL
const BOARD_CLOCK_HERTZ: u32 = 100_000_000;
const POLL_INTERVAL: Duration = Duration::from_millis(50);

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
    #[error("the programmed board does not run the compiled circuit")]
    Circuit,
}

struct View {
    contents: Contents,
    nodes: Vec<usize>,
}

impl View {
    fn new(contents: Contents, every_block: bool, circuit: &Circuit) -> View {
        let nodes = if every_block {
            (0..circuit.nodes().len()).collect()
        } else {
            circuit.io_nodes().to_vec()
        };
        View { contents, nodes }
    }

    fn read(
        &self,
        board: &mut Board,
        readback: &Readback,
    ) -> Result<impl Iterator<Item = (usize, State)> + '_, BoardError> {
        let payload = board.request(Command::Read(self.contents))?.payload;
        let states = readback.decode(self.contents, self.nodes.iter().copied(), &payload)?;
        Ok(self.nodes.iter().copied().zip(states))
    }
}

/// The board paces its own ticks; the host only polls its tick count and reads its states.
pub(crate) struct BoardEngine {
    board: Board,
    observer: Observer,
    readback: Readback,
    snapshot: View,
    report: View,
    circuit_clock_hertz: u32,
    rate: TickRate,
    target: u64,
    ticks_counted: u64,
}

impl BoardEngine {
    pub(crate) fn program(
        circuit: &Circuit,
        reporting: Reporting,
        progress: &dyn Fn(Stage),
    ) -> Result<BoardEngine, BoardError> {
        let port = std::env::var(PORT_VARIABLE).map_err(|_| BoardError::MissingPort)?;
        let directory = cache::unique_build_directory(&cache::board_root(), "", circuit.identity());
        progress(Stage::GenerateVerilog);
        let registers = Registers::new(circuit);
        rtl::write(&registers, reporting, &directory).map_err(BoardError::Write)?;
        progress(Stage::OpenBoardConnection);
        let mut board = Board::open(&port)?;
        let bitstream = toolchain::build(&directory, progress)?;
        progress(Stage::ProgramBoard);
        toolchain::program(&bitstream.path)?;
        progress(Stage::VerifyCircuit);
        board.discard_received()?;
        let status = board.request(Command::Status)?;
        if status.ticks != 0 || status.payload != circuit.identity().to_le_bytes() {
            return Err(BoardError::Circuit);
        }
        let mut engine = BoardEngine {
            board,
            observer: Observer::new(circuit, reporting),
            readback: Readback::new(&registers),
            snapshot: View::new(Contents::State, reporting.snapshots_every_block(), circuit),
            report: View::new(
                Contents::Appearance,
                reporting.reports_every_block_while_running(),
                circuit,
            ),
            circuit_clock_hertz: bitstream.circuit_clock_hertz,
            rate: TickRate::Paused,
            target: 0,
            ticks_counted: 0,
        };
        let initial = engine
            .snapshot
            .read(&mut engine.board, &engine.readback)?
            .all(|(node, state)| state == circuit.nodes()[node].initial);
        if !initial {
            return Err(BoardError::Circuit);
        }
        if let Err(error) = remove_build_artifacts(&directory) {
            tracing::warn!(%error, directory = %directory.display(), "could not remove the FPGA build artifacts");
        }
        Ok(engine)
    }
}

impl Engine for BoardEngine {
    fn set_tick_rate(&mut self, rate: TickRate) -> Result<(), EngineError> {
        let run = Command::run(rate, self.circuit_clock_hertz);
        let ticks = self.board.request(run)?.ticks;
        if let TickRate::Ticks(owed) = rate {
            self.target = ticks.saturating_add(owed);
        }
        self.rate = rate;
        Ok(())
    }

    fn set_send_rate(&mut self, _rate: SendRate) -> Result<(), EngineError> {
        Ok(())
    }

    fn run(&mut self, _deadline: Instant) -> Result<Progress, EngineError> {
        let ticks = self.board.request(Command::Status)?.ticks;
        let ticks_completed = ticks - self.ticks_counted;
        self.ticks_counted = ticks;
        let ticks_owed = match self.rate {
            TickRate::Ticks(_) => self.target - ticks,
            _ => 0,
        };
        let resume = match self.rate {
            TickRate::Paused => Resume::OnRequest,
            TickRate::Ticks(_) if ticks_owed == 0 => Resume::OnRequest,
            TickRate::Ticks(_) => Resume::Now,
            TickRate::Unlimited | TickRate::PerSecond(_) => Resume::After(POLL_INTERVAL),
        };
        Ok(Progress {
            ticks_completed,
            ticks_owed,
            resume,
        })
    }

    fn input(&mut self, node: NodeId, input: Input) -> Result<(), EngineError> {
        let (index, powered) = self.observer.input_power(node, input)?;
        let input = Command::Input {
            node: index as u32,
            powered,
        };
        self.board.request(input)?;
        Ok(())
    }

    fn take_changes(&mut self) -> Result<NodeChanges, EngineError> {
        let states = self.report.read(&mut self.board, &self.readback)?;
        Ok(self.observer.changes(states))
    }

    fn snapshot(&mut self) -> Result<EngineSnapshot, EngineError> {
        let states = self.snapshot.read(&mut self.board, &self.readback)?;
        Ok(self.observer.snapshot(states))
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
