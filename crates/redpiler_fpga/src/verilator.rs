//! Bytes pass at the frame level, without the two UARTs.

use crate::build_directory::BuildDirectory;
use crate::link::ByteLink;
use crate::process;
use crate::progress::Progress;
use crate::rtl::{self, CircuitSource, Sources};
use anyhow::Context;
use mchprs_simulation::backend::CompileError;
use std::io::{self, Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::sync::mpsc;
use std::thread::{self, JoinHandle};
use std::time::Duration;

const OUTPUT: &str = "verilated";
const EXECUTABLE: &str = "simulation";
const BUILD_TIMEOUT: Duration = Duration::from_secs(30 * 60);
const ANSWER_WALL_TIMEOUT: Duration = Duration::from_secs(60);

const BOARD_CLOCK_HERTZ: u64 = 100_000_000;
const WRITE: u8 = b'W';
const READ: u8 = b'R';

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Timing {
    pub board_half_period: u32,
    pub circuit_half_period: u32,
    pub clocks_per_byte: u32,
}

impl Timing {
    pub fn circuit_clock_hertz(self) -> f64 {
        BOARD_CLOCK_HERTZ as f64 * f64::from(self.board_half_period)
            / f64::from(self.circuit_half_period)
    }
}

#[derive(Debug)]
pub struct VerilatorExecutable {
    path: PathBuf,
    directory: BuildDirectory,
}

impl VerilatorExecutable {
    pub fn build(
        circuit: &CircuitSource,
        progress: &Progress,
    ) -> Result<VerilatorExecutable, CompileError> {
        let directory = BuildDirectory::new().context("could not create the build directory");
        let mut directory = directory.map_err(CompileError::Failed)?;
        let verilated = verilate(circuit, &directory.path, progress);
        verilated.map_err(|error| directory.keep_if_a_tool_failed(error))?;
        Ok(VerilatorExecutable {
            path: directory.path.join(OUTPUT).join(EXECUTABLE),
            directory,
        })
    }

    pub fn start(self, timing: Timing) -> io::Result<VerilatorLink> {
        let arguments = [
            timing.board_half_period,
            timing.circuit_half_period,
            timing.clocks_per_byte,
        ];
        let process = Command::new(&self.path)
            .args(arguments.map(|argument| argument.to_string()))
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()?;
        Ok(VerilatorLink::new(process, self.directory))
    }
}

fn verilate(
    circuit: &CircuitSource,
    directory: &Path,
    progress: &Progress,
) -> Result<(), CompileError> {
    let sources = Sources::write(directory, circuit, rtl::SIMULATION);
    let sources = sources
        .context("could not write the sources")
        .map_err(CompileError::Failed)?;
    let mut verilator = Command::new("verilator");
    verilator
        .current_dir(directory)
        .args(["--cc", "--exe", "--build", "-j", "0", "-Wno-PROCASSINIT"])
        .args(["--top-module", "top", "-Mdir", OUTPUT, "-o", EXECUTABLE])
        .args(sources.files);
    let log = directory.join("verilator.log");
    Ok(process::run(&mut verilator, &log, BUILD_TIMEOUT, progress)?)
}

/// A paused circuit advances only for requests; an active circuit also clocks between them.
#[derive(Debug)]
pub struct VerilatorLink {
    process: Child,
    requests: ChildStdin,
    reads: Option<mpsc::Sender<usize>>,
    answers: mpsc::Receiver<io::Result<Vec<u8>>>,
    reader: Option<JoinHandle<()>>,
    _directory: BuildDirectory,
}

impl VerilatorLink {
    fn new(mut process: Child, directory: BuildDirectory) -> VerilatorLink {
        let requests = process.stdin.take().expect("stdin is piped");
        let mut output = process.stdout.take().expect("stdout is piped");
        let (reads, pending_reads) = mpsc::channel();
        let (answers, received_answers) = mpsc::channel();
        let reader = thread::spawn(move || {
            for capacity in pending_reads {
                if answers.send(read_answer(&mut output, capacity)).is_err() {
                    break;
                }
            }
        });
        VerilatorLink {
            process,
            requests,
            reads: Some(reads),
            answers: received_answers,
            reader: Some(reader),
            _directory: directory,
        }
    }

    fn stop(&mut self) {
        self.reads.take();
        let _ = self.process.kill();
        let _ = self.process.wait();
        if let Some(reader) = self.reader.take() {
            let _ = reader.join();
        }
    }
}

fn read_answer(output: &mut ChildStdout, capacity: usize) -> io::Result<Vec<u8>> {
    let mut count = [0; 4];
    output.read_exact(&mut count)?;
    let count = u32::from_ne_bytes(count) as usize;
    if count > capacity {
        return Err(io::ErrorKind::InvalidData.into());
    }
    let mut bytes = vec![0; count];
    output.read_exact(&mut bytes)?;
    Ok(bytes)
}

impl ByteLink for VerilatorLink {
    fn write(&mut self, bytes: &[u8]) -> io::Result<()> {
        let length = u32::try_from(bytes.len()).map_err(io::Error::other)?;
        let message = [&[WRITE], &length.to_ne_bytes()[..], bytes].concat();
        self.requests.write_all(&message)
    }

    fn read(&mut self, buffer: &mut [u8], timeout: Duration) -> io::Result<usize> {
        let clocks = timeout.as_nanos() * u128::from(BOARD_CLOCK_HERTZ) / 1_000_000_000;
        let clocks = u64::try_from(clocks).unwrap_or(u64::MAX);
        let capacity = u32::try_from(buffer.len()).unwrap_or(u32::MAX);
        let message = [&[READ], &clocks.to_ne_bytes()[..], &capacity.to_ne_bytes()].concat();
        self.requests.write_all(&message)?;
        let reads = self.reads.as_ref().ok_or(io::ErrorKind::BrokenPipe)?;
        reads
            .send(capacity as usize)
            .map_err(|_| io::ErrorKind::BrokenPipe)?;
        let received = match self.answers.recv_timeout(ANSWER_WALL_TIMEOUT) {
            Ok(answer) => answer?,
            Err(error) => {
                self.stop();
                return Err(match error {
                    mpsc::RecvTimeoutError::Timeout => io::ErrorKind::TimedOut,
                    mpsc::RecvTimeoutError::Disconnected => io::ErrorKind::BrokenPipe,
                }
                .into());
            }
        };
        if received.is_empty() {
            return Err(io::ErrorKind::TimedOut.into());
        }
        buffer[..received.len()].copy_from_slice(&received);
        Ok(received.len())
    }
}

impl Drop for VerilatorLink {
    fn drop(&mut self) {
        self.stop();
    }
}
