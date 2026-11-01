mod compile;

use crate::circuit::{Circuit, State};
use crate::engine::Simulator;
use crate::readback::{Readback, ReadbackError};
use crate::registers::Registers;
use crate::stage::Stage;
use crate::subprocess::{Process, SubprocessError};
use std::fs::File;
use std::io::{self, BufReader, Read, Write};
use std::path::{Path, PathBuf};
use std::process::{ChildStdin, Command, Stdio};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError};
use std::thread::{self, JoinHandle};
use std::time::Duration;

const STALL_TIMEOUT: Duration = Duration::from_secs(30);
const RUNTIME_LOG: &str = "verilator-runtime.log";
const PROGRESS_MARKER: &[u8; 4] = b"MCHG";
const SNAPSHOT_MARKER: &[u8; 4] = b"MCHP";
const INPUT_REQUEST: u8 = b'I';
const STEP_REQUEST: u8 = b'S';
const TICKS_BYTES: usize = size_of::<u64>();

#[derive(Debug, thiserror::Error)]
pub(crate) enum VerilatorError {
    #[error("could not prepare the Verilator circuit")]
    Io(#[from] io::Error),
    #[error("verilator --version failed with {0}")]
    Version(std::process::ExitStatus),
    #[error("Verilator build failed; see {}", .log.display())]
    Build {
        log: PathBuf,
        #[source]
        source: SubprocessError,
    },
    #[error("could not start the Verilator circuit")]
    Start(#[source] SubprocessError),
    #[error("Verilator circuit failed; see {}", .log.display())]
    Failed {
        log: PathBuf,
        #[source]
        source: RuntimeError,
    },
    #[error("Verilator circuit stopped after a previous failure; see {}", .log.display())]
    Stopped { log: PathBuf },
}

#[derive(Debug, thiserror::Error)]
pub(crate) enum RuntimeError {
    #[error("Verilator circuit pipe failed")]
    Pipe(#[from] io::Error),
    #[error("Verilator circuit sent the invalid marker {0:?}")]
    Marker([u8; 4]),
    #[error("Verilator circuit made no progress for {} seconds", .0.as_secs())]
    Stalled(Duration),
    #[error("Verilator circuit disconnected")]
    Disconnected,
    #[error("Verilator snapshot is at tick {actual} instead of {expected}")]
    Ticks { actual: u64, expected: u64 },
    #[error(transparent)]
    Readback(#[from] ReadbackError),
}

enum Reply {
    Progress,
    Snapshot(Vec<u8>),
}

fn read_reply(output: &mut impl Read, packet_size: usize) -> Result<Reply, RuntimeError> {
    let mut marker = [0; 4];
    output.read_exact(&mut marker)?;
    match &marker {
        PROGRESS_MARKER => Ok(Reply::Progress),
        SNAPSHOT_MARKER => {
            let mut packet = vec![0; packet_size];
            output.read_exact(&mut packet)?;
            Ok(Reply::Snapshot(packet))
        }
        _ => Err(RuntimeError::Marker(marker)),
    }
}

struct Running {
    process: Process,
    input: ChildStdin,
    replies: Receiver<Result<Reply, RuntimeError>>,
    reader: JoinHandle<()>,
}

impl Running {
    fn start(directory: &Path, packet_size: usize) -> Result<Self, VerilatorError> {
        let mut process = Process::spawn(
            Command::new(compile::simulator(directory))
                .current_dir(directory)
                .stdin(Stdio::piped())
                .stdout(Stdio::piped())
                .stderr(File::create(directory.join(RUNTIME_LOG))?),
        )
        .map_err(VerilatorError::Start)?;
        let input = process.take_stdin().expect("stdin is piped");
        let output = process.take_stdout().expect("stdout is piped");
        let (sender, replies) = mpsc::sync_channel(1);
        let reader = thread::Builder::new()
            .name("verilator-snapshot".into())
            .spawn(move || {
                let mut output = BufReader::new(output);
                loop {
                    let result = read_reply(&mut output, packet_size);
                    let failed = result.is_err();
                    if sender.send(result).is_err() || failed {
                        break;
                    }
                }
            })?;
        Ok(Self {
            process,
            input,
            replies,
            reader,
        })
    }

    fn exchange(&mut self, request: Option<&[u8]>) -> Result<Vec<u8>, RuntimeError> {
        if let Some(request) = request {
            self.input.write_all(request)?;
            self.input.flush()?;
        }
        self.next_snapshot()
    }

    fn next_snapshot(&self) -> Result<Vec<u8>, RuntimeError> {
        loop {
            let reply =
                self.replies
                    .recv_timeout(STALL_TIMEOUT)
                    .map_err(|error| match error {
                        RecvTimeoutError::Timeout => RuntimeError::Stalled(STALL_TIMEOUT),
                        RecvTimeoutError::Disconnected => RuntimeError::Disconnected,
                    })??;
            if let Reply::Snapshot(packet) = reply {
                return Ok(packet);
            }
        }
    }

    fn stop(self) {
        let Self {
            mut process,
            input,
            replies,
            reader,
        } = self;
        drop(input);
        drop(replies);
        process.terminate();
        let _ = reader.join();
    }
}

pub(crate) struct Verilator {
    running: Option<Running>,
    log: PathBuf,
    readback: Readback,
    node_count: usize,
    ticks: u64,
    states: Vec<State>,
}

impl Verilator {
    pub(crate) fn compile(
        circuit: &Circuit,
        progress: &dyn Fn(Stage),
    ) -> Result<Self, VerilatorError> {
        let registers = Registers::new(circuit);
        let readback = Readback::new(&registers);
        let node_count = circuit.nodes().len();
        let state_bytes = readback.bytes(0..node_count);
        let directory = compile::cached_simulator(&registers, state_bytes, progress)?;
        progress(Stage::StartVerilatorCircuit);
        let mut verilator = Self {
            running: Some(Running::start(&directory, TICKS_BYTES + state_bytes)?),
            log: directory.join(RUNTIME_LOG),
            readback,
            node_count,
            ticks: 0,
            states: Vec::new(),
        };
        verilator.exchange(None, 0)?;
        Ok(verilator)
    }

    fn exchange(&mut self, request: Option<&[u8]>, expected: u64) -> Result<(), VerilatorError> {
        let Some(running) = self.running.as_mut() else {
            return Err(VerilatorError::Stopped {
                log: self.log.clone(),
            });
        };
        let result = match running.exchange(request) {
            Ok(packet) => self.decode(&packet, expected),
            Err(error) => Err(error),
        };
        match result {
            Ok(states) => {
                self.ticks = expected;
                self.states = states;
                Ok(())
            }
            Err(source) => {
                if let Some(running) = self.running.take() {
                    running.stop();
                }
                Err(VerilatorError::Failed {
                    log: self.log.clone(),
                    source,
                })
            }
        }
    }

    fn decode(&self, packet: &[u8], expected: u64) -> Result<Vec<State>, RuntimeError> {
        let (ticks, states) = packet.split_at(TICKS_BYTES);
        let actual = u64::from_le_bytes(ticks.try_into().expect("eight tick bytes"));
        if actual != expected {
            return Err(RuntimeError::Ticks { actual, expected });
        }
        Ok(self.readback.decode(0..self.node_count, states)?)
    }
}

impl Drop for Verilator {
    fn drop(&mut self) {
        if let Some(running) = self.running.take() {
            running.stop();
        }
    }
}

impl Simulator for Verilator {
    type Error = VerilatorError;

    fn input(&mut self, node: usize, powered: bool) -> Result<(), VerilatorError> {
        let mut request = [INPUT_REQUEST, 0, 0, 0, 0, u8::from(powered)];
        request[1..5].copy_from_slice(&(node as u32).to_le_bytes());
        self.exchange(Some(&request), self.ticks)
    }

    fn advance(&mut self, ticks: u64) -> Result<(), VerilatorError> {
        let mut request = [STEP_REQUEST; 9];
        request[1..].copy_from_slice(&ticks.to_le_bytes());
        self.exchange(Some(&request), self.ticks + ticks)
    }

    fn states(&mut self) -> Result<Vec<State>, VerilatorError> {
        Ok(self.states.clone())
    }
}
