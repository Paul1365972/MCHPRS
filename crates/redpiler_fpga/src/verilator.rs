mod process;

use self::process::Process;
use crate::circuit::{Circuit, Kind, Snapshot, State};
use crate::engine::Simulator;
use crate::registers::Readback;
use sha2::{Digest, Sha256};
use std::fs::{self, File};
use std::io::{self, BufReader, Read, Write};
use std::path::{Path, PathBuf};
use std::process::{ChildStdin, Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

const BUILD_TIMEOUT: Duration = Duration::from_secs(3 * 60 * 60);
const STALL_TIMEOUT: Duration = Duration::from_secs(30);
const COMPILE_MEMORY_BASE: u64 = 512 << 20;
const COMPILE_MEMORY_PER_NODE: u64 = 16 << 10;
const SIMULATOR: &str = if cfg!(windows) {
    "simulator.exe"
} else {
    "simulator"
};
const VERILATOR_ARGUMENTS: &[&str] = &[
    "--cc",
    "--exe",
    "--top-module",
    "compiled_circuit",
    "--prefix",
    "Vcompiled_circuit",
    "--Mdir",
    "obj_dir",
    "-o",
    SIMULATOR,
    "-Wno-fatal",
    "-fno-dfg",
    "--output-split",
    "20000",
    "--output-split-cfuncs",
    "500",
    "--output-groups",
    "0",
];

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

fn cache_root() -> PathBuf {
    match std::env::var_os("MCHPRS_FPGA_CACHE") {
        Some(root) => PathBuf::from(root).join("verilator"),
        None => std::env::temp_dir().join("mchprs-verilator"),
    }
}

fn available_memory() -> Option<u64> {
    let information = fs::read_to_string("/proc/meminfo").ok()?;
    let kibibytes = information
        .lines()
        .find_map(|line| line.strip_prefix("MemAvailable:"))?
        .trim()
        .strip_suffix("kB")?
        .trim()
        .parse::<u64>()
        .ok()?;
    Some(kibibytes << 10)
}

fn compile_jobs(node_count: usize) -> usize {
    let processors = thread::available_parallelism().map_or(1, |jobs| jobs.get());
    let memory_per_job = COMPILE_MEMORY_BASE + node_count as u64 * COMPILE_MEMORY_PER_NODE;
    match available_memory() {
        Some(memory) => usize::try_from(memory / memory_per_job)
            .unwrap_or(usize::MAX)
            .clamp(1, processors),
        None => processors,
    }
}

fn verilator_version() -> io::Result<Vec<u8>> {
    let output = Command::new("verilator").arg("--version").output()?;
    if output.status.success() {
        Ok(output.stdout)
    } else {
        Err(io::Error::other(format!(
            "verilator --version failed with {}",
            output.status
        )))
    }
}

fn build_key(directory: &Path, sources: &[PathBuf]) -> io::Result<String> {
    let mut hasher = Sha256::new();
    hasher.update(verilator_version()?);
    for argument in VERILATOR_ARGUMENTS {
        hasher.update(argument.as_bytes());
        hasher.update([0]);
    }
    for source in sources {
        hasher.update(source.to_string_lossy().as_bytes());
        hasher.update([0]);
        hasher.update(fs::read(directory.join(source))?);
    }
    Ok(hasher
        .finalize()
        .iter()
        .take(16)
        .map(|byte| format!("{byte:02x}"))
        .collect())
}

fn run_build_step(command: &mut Command, log: &Path, started: Instant) -> io::Result<()> {
    let output = File::options().create(true).append(true).open(log)?;
    command
        .stdin(Stdio::null())
        .stdout(output.try_clone()?)
        .stderr(output);
    let mut process = Process::spawn(command)?;
    loop {
        if started.elapsed() >= BUILD_TIMEOUT {
            return Err(io::Error::new(
                io::ErrorKind::TimedOut,
                "Verilator build timed out",
            ));
        }
        if let Some(status) = process.0.try_wait()? {
            return if status.success() {
                Ok(())
            } else {
                Err(io::Error::other(format!(
                    "Verilator build failed with {status}"
                )))
            };
        }
        thread::sleep(Duration::from_millis(100));
    }
}

fn build(
    directory: &Path,
    sources: &[PathBuf],
    node_count: usize,
    state_bytes: usize,
    progress: &dyn Fn(&'static str),
) -> io::Result<()> {
    let log = directory.join("verilator-build.log");
    let started = Instant::now();
    let result = (|| {
        progress("Generating Verilator model");
        run_build_step(
            Command::new("verilator")
                .args(VERILATOR_ARGUMENTS)
                .arg("-CFLAGS")
                .arg(format!(
                    "-std=c++17 -DMCHPRS_NODE_COUNT={node_count} -DMCHPRS_STATE_BYTES={state_bytes}"
                ))
                .args(sources)
                .current_dir(directory),
            &log,
            started,
        )?;
        progress("Compiling Verilator model");
        run_build_step(
            Command::new("make")
                .args(["-f", "Vcompiled_circuit.mk", "-j"])
                .arg(compile_jobs(node_count).to_string())
                .arg(SIMULATOR)
                .current_dir(directory.join("obj_dir")),
            &log,
            started,
        )
    })();
    result.map_err(|error| io::Error::new(error.kind(), format!("{error}; see {}", log.display())))
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
        static SEQUENCE: AtomicU64 = AtomicU64::new(0);
        let root = cache_root();
        let staging = root.join(format!(
            "staging-{:016x}-{}-{}",
            circuit.identity(),
            std::process::id(),
            SEQUENCE.fetch_add(1, Ordering::Relaxed)
        ));
        progress("Writing Verilator circuit");
        let readback = Readback::new(circuit);
        let node_count = circuit.nodes().len();
        let state_bytes = readback.bytes(0..node_count);
        circuit.write_simulation(&staging)?;
        fs::write(
            staging.join("runner.cpp"),
            include_str!("verilator/runner.cpp"),
        )?;
        let mut sources: Vec<_> = crate::rtl::RTL
            .find("**/*.v")
            .expect("valid Verilog glob")
            .map(|entry| Path::new("rtl").join(entry.path()))
            .collect();
        sources.sort();
        sources.push(PathBuf::from("circuit.v"));
        sources.push(PathBuf::from("runner.cpp"));
        let directory = root.join(build_key(&staging, &sources)?);
        let simulator = directory.join("obj_dir").join(SIMULATOR);
        if simulator.exists() {
            fs::remove_dir_all(&staging)?;
        } else {
            build(&staging, &sources, node_count, state_bytes, progress)?;
            if let Err(error) = fs::rename(&staging, &directory) {
                if !simulator.exists() {
                    return Err(error);
                }
                fs::remove_dir_all(&staging)?;
            }
        }
        let directory = directory.canonicalize()?;
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
                .map(|node| matches!(node.kind, Kind::Lever | Kind::Button | Kind::PressurePlate))
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
