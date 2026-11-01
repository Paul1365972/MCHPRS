mod compile;
mod process;

use crate::circuit::{Circuit, Snapshot, State};
use crate::engine::Simulator;
use crate::readback::Readback;
use crate::registers::Registers;
use compile::SIMULATOR;
use process::Process;
use std::fs::File;
use std::io::{self, BufReader, Read, Write};
use std::path::PathBuf;
use std::process::{ChildStdin, Command, Stdio};
use std::sync::mpsc::{self, Receiver};
use std::thread::{self, JoinHandle};
use std::time::Duration;

const STALL_TIMEOUT: Duration = Duration::from_secs(30);

enum Reply {
    Progress,
    Snapshot(Vec<u8>),
}

fn read_reply(output: &mut impl Read, packet_size: usize) -> io::Result<Reply> {
    let mut marker = [0; 4];
    output.read_exact(&mut marker)?;
    match &marker {
        b"MCHG" => Ok(Reply::Progress),
        b"MCHP" => {
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

pub(crate) struct VerilatorEngine {
    process: Option<Process>,
    input: ChildStdin,
    replies: Option<Receiver<io::Result<Reply>>>,
    reader: Option<JoinHandle<()>>,
    directory: PathBuf,
    inputs: Vec<bool>,
    readback: Readback,
    node_count: usize,
    snapshot: Snapshot,
}

impl VerilatorEngine {
    pub(crate) fn compile(circuit: &Circuit, progress: &dyn Fn(&'static str)) -> io::Result<Self> {
        let registers = Registers::new(circuit);
        let readback = Readback::new(&registers);
        let node_count = circuit.nodes().len();
        let state_bytes = readback.bytes(0..node_count);
        let directory = compile::cached_simulator(&registers, state_bytes, progress)?;
        progress("Starting Verilator circuit");
        let log = directory.join("verilator-runtime.log");
        let mut process = Process::spawn(
            Command::new(directory.join("obj_dir").join(SIMULATOR))
                .current_dir(&directory)
                .stdin(Stdio::piped())
                .stdout(Stdio::piped())
                .stderr(File::create(&log)?),
        )?;
        let input = process
            .0
            .stdin
            .take()
            .ok_or_else(|| io::Error::other("missing simulator stdin"))?;
        let output = process
            .0
            .stdout
            .take()
            .ok_or_else(|| io::Error::other("missing simulator stdout"))?;
        let packet_size = 8 + state_bytes;
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
        let mut engine = Self {
            process: Some(process),
            input,
            replies: Some(replies),
            reader: Some(reader),
            directory,
            inputs: circuit
                .nodes()
                .iter()
                .map(|node| node.kind.is_input())
                .collect(),
            readback,
            node_count,
            snapshot: Snapshot {
                ticks: 0,
                states: vec![State::default(); node_count],
            },
        };
        engine.receive(0)?;
        Ok(engine)
    }

    fn send_input(&mut self, index: usize, value: bool) -> io::Result<()> {
        if !self.inputs.get(index).copied().unwrap_or(false) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "node is not an input kind or index is out of bounds",
            ));
        }
        let mut request = [b'I', 0, 0, 0, 0, u8::from(value)];
        request[1..5].copy_from_slice(&(index as u32).to_le_bytes());
        self.request(&request, self.snapshot.ticks)
    }

    fn step(&mut self, ticks: u64) -> io::Result<()> {
        let expected = self.snapshot.ticks.checked_add(ticks).ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "Verilator tick counter overflow",
            )
        })?;
        let mut request = [b'S'; 9];
        request[1..].copy_from_slice(&ticks.to_le_bytes());
        self.request(&request, expected)
    }

    fn request(&mut self, request: &[u8], expected: u64) -> io::Result<()> {
        if self.process.is_none() {
            return Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "Verilator engine failed",
            ));
        }
        if let Err(error) = self
            .input
            .write_all(request)
            .and_then(|()| self.input.flush())
        {
            return Err(self.fail(error));
        }
        self.receive(expected)
    }

    fn receive(&mut self, expected: u64) -> io::Result<()> {
        let result = (|| {
            let replies = self.replies.as_ref().ok_or_else(|| {
                io::Error::new(io::ErrorKind::BrokenPipe, "Verilator engine failed")
            })?;
            let packet = loop {
                match replies
                    .recv_timeout(STALL_TIMEOUT)
                    .map_err(|error| match error {
                        mpsc::RecvTimeoutError::Timeout => io::Error::new(
                            io::ErrorKind::TimedOut,
                            "Verilator circuit made no progress for 30 seconds",
                        ),
                        mpsc::RecvTimeoutError::Disconnected => io::Error::new(
                            io::ErrorKind::BrokenPipe,
                            "Verilator circuit disconnected",
                        ),
                    })?? {
                    Reply::Progress => {}
                    Reply::Snapshot(packet) => break packet,
                }
            };
            let ticks = u64::from_le_bytes(packet[..8].try_into().unwrap());
            if ticks != expected {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "invalid Verilator snapshot tick count",
                ));
            }
            let states = self.readback.decode(0..self.node_count, &packet[8..])?;
            self.snapshot = Snapshot { ticks, states };
            Ok(())
        })();
        result.map_err(|error| self.fail(error))
    }

    fn fail(&mut self, error: io::Error) -> io::Error {
        self.replies.take();
        if let Some(mut process) = self.process.take() {
            process.terminate();
        }
        io::Error::new(
            error.kind(),
            format!(
                "{error}; Verilator engine is unusable; see {}",
                self.directory.join("verilator-runtime.log").display()
            ),
        )
    }
}

impl Drop for VerilatorEngine {
    fn drop(&mut self) {
        self.replies.take();
        if let Some(mut process) = self.process.take() {
            process.terminate();
        }
        if let Some(reader) = self.reader.take() {
            let _ = reader.join();
        }
    }
}

impl Simulator for VerilatorEngine {
    fn input(&mut self, node: usize, powered: bool) -> io::Result<()> {
        self.send_input(node, powered)
    }

    fn advance(&mut self, ticks: u64) -> io::Result<()> {
        self.step(ticks)
    }

    fn states(&mut self) -> io::Result<Vec<State>> {
        Ok(self.snapshot.states.clone())
    }
}
