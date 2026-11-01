mod compile;

use crate::circuit::{Circuit, State};
use crate::engine::Simulator;
use crate::readback::Readback;
use crate::registers::Registers;
use crate::subprocess::Process;
use std::fs::File;
use std::io::{self, BufReader, Read, Write};
use std::path::{Path, PathBuf};
use std::process::{ChildStdin, Command, Stdio};
use std::sync::mpsc::{self, Receiver};
use std::thread::{self, JoinHandle};
use std::time::Duration;

const STALL_TIMEOUT: Duration = Duration::from_secs(30);
const RUNTIME_LOG: &str = "verilator-runtime.log";
const PROGRESS_MARKER: &[u8; 4] = b"MCHG";
const SNAPSHOT_MARKER: &[u8; 4] = b"MCHP";
const INPUT_REQUEST: u8 = b'I';
const STEP_REQUEST: u8 = b'S';
const TICKS_BYTES: usize = size_of::<u64>();

enum Reply {
    Progress,
    Snapshot(Vec<u8>),
}

fn read_reply(output: &mut impl Read, packet_size: usize) -> io::Result<Reply> {
    let mut marker = [0; 4];
    output.read_exact(&mut marker)?;
    match &marker {
        PROGRESS_MARKER => Ok(Reply::Progress),
        SNAPSHOT_MARKER => {
            let mut packet = vec![0; packet_size];
            output.read_exact(&mut packet)?;
            Ok(Reply::Snapshot(packet))
        }
        _ => Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "invalid Verilator message marker",
        )),
    }
}

struct Running {
    process: Process,
    input: ChildStdin,
    replies: Receiver<io::Result<Reply>>,
    reader: JoinHandle<()>,
}

impl Running {
    fn start(directory: &Path, packet_size: usize) -> io::Result<Self> {
        let mut process = Process::spawn(
            Command::new(compile::simulator(directory))
                .current_dir(directory)
                .stdin(Stdio::piped())
                .stdout(Stdio::piped())
                .stderr(File::create(directory.join(RUNTIME_LOG))?),
        )
        .map_err(io::Error::other)?;
        let input = process
            .take_stdin()
            .ok_or_else(|| io::Error::other("missing simulator stdin"))?;
        let output = process
            .take_stdout()
            .ok_or_else(|| io::Error::other("missing simulator stdout"))?;
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

    fn exchange(&mut self, request: Option<&[u8]>) -> io::Result<Vec<u8>> {
        if let Some(request) = request {
            self.input.write_all(request)?;
            self.input.flush()?;
        }
        self.next_snapshot()
    }

    fn next_snapshot(&self) -> io::Result<Vec<u8>> {
        loop {
            let reply =
                self.replies
                    .recv_timeout(STALL_TIMEOUT)
                    .map_err(|error| match error {
                        mpsc::RecvTimeoutError::Timeout => io::Error::new(
                            io::ErrorKind::TimedOut,
                            format!(
                                "Verilator circuit made no progress for {} seconds",
                                STALL_TIMEOUT.as_secs()
                            ),
                        ),
                        mpsc::RecvTimeoutError::Disconnected => io::Error::new(
                            io::ErrorKind::BrokenPipe,
                            "Verilator circuit disconnected",
                        ),
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
    pub(crate) fn compile(circuit: &Circuit, progress: &dyn Fn(&'static str)) -> io::Result<Self> {
        let registers = Registers::new(circuit);
        let readback = Readback::new(&registers);
        let node_count = circuit.nodes().len();
        let state_bytes = readback.bytes(0..node_count);
        let directory = compile::cached_simulator(&registers, state_bytes, progress)?;
        progress("Starting Verilator circuit");
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

    fn exchange(&mut self, request: Option<&[u8]>, expected: u64) -> io::Result<()> {
        let running = self
            .running
            .as_mut()
            .ok_or_else(|| io::Error::new(io::ErrorKind::BrokenPipe, "Verilator engine failed"))?;
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
            Err(error) => Err(self.fail(error)),
        }
    }

    fn decode(&self, packet: &[u8], expected: u64) -> io::Result<Vec<State>> {
        let (ticks, states) = packet.split_at(TICKS_BYTES);
        if ticks != expected.to_le_bytes() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "invalid Verilator snapshot tick count",
            ));
        }
        self.readback.decode(0..self.node_count, states)
    }

    fn fail(&mut self, error: io::Error) -> io::Error {
        if let Some(running) = self.running.take() {
            running.stop();
        }
        io::Error::new(
            error.kind(),
            format!(
                "{error}; Verilator engine is unusable; see {}",
                self.log.display()
            ),
        )
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
    fn input(&mut self, node: usize, powered: bool) -> io::Result<()> {
        let mut request = [INPUT_REQUEST, 0, 0, 0, 0, u8::from(powered)];
        request[1..5].copy_from_slice(&(node as u32).to_le_bytes());
        self.exchange(Some(&request), self.ticks)
    }

    fn advance(&mut self, ticks: u64) -> io::Result<()> {
        let mut request = [STEP_REQUEST; 9];
        request[1..].copy_from_slice(&ticks.to_le_bytes());
        self.exchange(Some(&request), self.ticks + ticks)
    }

    fn states(&mut self) -> io::Result<Vec<State>> {
        Ok(self.states.clone())
    }
}
