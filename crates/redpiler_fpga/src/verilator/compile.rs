use super::VerilatorError;
use crate::cache;
use crate::registers::Registers;
use crate::rtl;
use crate::stage::Stage;
use crate::subprocess;
use sha2::{Digest, Sha256};
use std::fs::{self, File};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::thread;
use std::time::{Duration, Instant};

const BUILD_TIMEOUT: Duration = Duration::from_secs(3 * 60 * 60);
const COMPILE_MEMORY_BASE: u64 = 512 << 20;
const COMPILE_MEMORY_PER_NODE: u64 = 16 << 10;
const STAGING_PREFIX: &str = "staging-";
const BUILD_LOG: &str = "verilator-build.log";
const RUNNER: &str = "runner.cpp";
const MODEL_PREFIX: &str = "Vcompiled_circuit";
const OBJECT_DIRECTORY: &str = "obj_dir";
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
    MODEL_PREFIX,
    "--Mdir",
    OBJECT_DIRECTORY,
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

pub(super) fn simulator(directory: &Path) -> PathBuf {
    directory.join(OBJECT_DIRECTORY).join(SIMULATOR)
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

fn verilator_version() -> Result<Vec<u8>, VerilatorError> {
    let output = Command::new("verilator").arg("--version").output()?;
    if output.status.success() {
        Ok(output.stdout)
    } else {
        Err(VerilatorError::Version(output.status))
    }
}

fn build_key(directory: &Path, sources: &[PathBuf]) -> Result<String, VerilatorError> {
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

fn run_build_step(
    command: &mut Command,
    log: &Path,
    deadline: Instant,
) -> Result<(), VerilatorError> {
    let output = File::options().create(true).append(true).open(log)?;
    command
        .stdin(Stdio::null())
        .stdout(output.try_clone()?)
        .stderr(output);
    subprocess::run(command, deadline).map_err(|source| VerilatorError::Build {
        log: log.to_owned(),
        source,
    })
}

fn build(
    directory: &Path,
    sources: &[PathBuf],
    node_count: usize,
    state_bytes: usize,
    progress: &dyn Fn(Stage),
) -> Result<(), VerilatorError> {
    let log = directory.join(BUILD_LOG);
    let deadline = Instant::now() + BUILD_TIMEOUT;
    progress(Stage::GenerateVerilatorModel);
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
        deadline,
    )?;
    progress(Stage::CompileVerilatorModel);
    run_build_step(
        Command::new("make")
            .arg("-f")
            .arg(format!("{MODEL_PREFIX}.mk"))
            .arg("-j")
            .arg(compile_jobs(node_count).to_string())
            .arg(SIMULATOR)
            .current_dir(directory.join(OBJECT_DIRECTORY)),
        &log,
        deadline,
    )
}

pub(super) fn cached_simulator(
    registers: &Registers,
    state_bytes: usize,
    progress: &dyn Fn(Stage),
) -> Result<PathBuf, VerilatorError> {
    let circuit = registers.circuit;
    let root = cache::verilator_root();
    let staging = cache::unique_build_directory(&root, STAGING_PREFIX, circuit.identity());
    progress(Stage::WriteVerilatorCircuit);
    rtl::write(registers, &staging)?;
    fs::write(staging.join(RUNNER), include_str!("runner.cpp"))?;
    let mut sources = rtl::sources();
    sources.push(PathBuf::from(RUNNER));
    let directory = root.join(build_key(&staging, &sources)?);
    let simulator = simulator(&directory);
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
                return Err(error.into());
            }
            fs::remove_dir_all(&staging)?;
        }
    }
    Ok(directory.canonicalize()?)
}
