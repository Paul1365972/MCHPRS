use super::process::Process;
use crate::registers::Registers;
use crate::rtl::{self, RTL};
use sha2::{Digest, Sha256};
use std::fs::{self, File};
use std::io;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::thread;
use std::time::{Duration, Instant};

const BUILD_TIMEOUT: Duration = Duration::from_secs(3 * 60 * 60);
const COMPILE_MEMORY_BASE: u64 = 512 << 20;
const COMPILE_MEMORY_PER_NODE: u64 = 16 << 10;
pub(super) const SIMULATOR: &str = if cfg!(windows) {
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
    "-Wno-PINMISSING",
    "-fno-dfg",
    "--output-split",
    "20000",
    "--output-split-cfuncs",
    "500",
    "--output-groups",
    "0",
];

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

pub(super) fn cached_simulator(
    registers: &Registers,
    state_bytes: usize,
    progress: &dyn Fn(&'static str),
) -> io::Result<PathBuf> {
    static SEQUENCE: AtomicU64 = AtomicU64::new(0);
    let circuit = registers.circuit;
    let root = cache_root();
    let staging = root.join(format!(
        "staging-{:016x}-{}-{}",
        circuit.identity(),
        std::process::id(),
        SEQUENCE.fetch_add(1, Ordering::Relaxed)
    ));
    progress("Writing Verilator circuit");
    rtl::write(registers, &staging)?;
    fs::write(staging.join("runner.cpp"), include_str!("runner.cpp"))?;
    let mut sources: Vec<_> = RTL
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
        build(
            &staging,
            &sources,
            circuit.nodes().len(),
            state_bytes,
            progress,
        )?;
        if let Err(error) = fs::rename(&staging, &directory) {
            if !simulator.exists() {
                return Err(error);
            }
            fs::remove_dir_all(&staging)?;
        }
    }
    directory.canonicalize()
}
