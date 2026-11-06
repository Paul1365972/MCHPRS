use crate::cache;
use crate::rtl;
use crate::stage::Stage;
use crate::subprocess::{self, SubprocessError};
use serde::Deserialize;
use std::collections::HashMap;
use std::env;
use std::fs::{self, File};
use std::io;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

const DEVICE: &str = "xc7a35tcpg236-1";
// The XC7A35T shares the XC7A50T die, so nextpnr routes it with the XC7A50T chip database.
const CHIP_DATABASE_DEVICE: &str = "xc7a50t";
/// The divisors of the board clock that the system clock can take, fastest first
const CLOCK_DIVISORS: [u32; 2] = [1, 2];
/// The fewest flip-flops an enable or reset must drive to keep its dedicated pin
const CONTROL_SET_FLIP_FLOPS: u32 = 4;
const BUILD_TIMEOUT: Duration = Duration::from_secs(30 * 60);
const ASSEMBLY_TIMEOUT: Duration = Duration::from_secs(10 * 60);
const CONVERSION_TIMEOUT: Duration = Duration::from_secs(5 * 60);
const PROGRAM_TIMEOUT: Duration = Duration::from_secs(60);
const LOCK_TIMEOUT: Duration = Duration::from_secs(45 * 60);

#[derive(Debug, thiserror::Error)]
pub(crate) enum ToolchainError {
    #[error("{0} is not set; enter an openXC7 development environment")]
    Environment(&'static str),
    #[error("Yosys paths must be UTF-8 without line breaks: {}", .0.display())]
    YosysPath(PathBuf),
    #[error("{} has no parent directory", .0.display())]
    Orphan(PathBuf),
    #[error("could not access the FPGA build files")]
    Io(#[from] io::Error),
    #[error("could not lock the FPGA chip database {}", .path.display())]
    Lock {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("FPGA {stage} needs {program}; enter the FPGA Nix development shell")]
    MissingTool {
        stage: &'static str,
        program: String,
        #[source]
        source: io::Error,
    },
    #[error("could not read the timing report {}", .path.display())]
    Report {
        path: PathBuf,
        #[source]
        source: serde_json::Error,
    },
    #[error("the timing report names no clock")]
    Unclocked,
    #[error("the circuit reaches {achieved_megahertz:.1} MHz, but the slowest system clock is {constraint_megahertz} MHz")]
    Timing {
        achieved_megahertz: f64,
        constraint_megahertz: f64,
    },
    #[error("FPGA {stage} failed; see {} and {}", .stdout.display(), .stderr.display())]
    Stage {
        stage: &'static str,
        stdout: PathBuf,
        stderr: PathBuf,
        #[source]
        source: SubprocessError,
    },
}

pub(super) struct Bitstream {
    pub(super) path: PathBuf,
    pub(super) clock_divisor: u32,
}

pub(super) fn build(
    directory: &Path,
    progress: &dyn Fn(Stage),
) -> Result<Bitstream, ToolchainError> {
    let directory = directory.canonicalize()?;
    let cache = parent(&directory)?;
    let database = environment_path("PRJXRAY_DB_DIR")?.join("artix7");
    let nextpnr = environment_path("NEXTPNR_XILINX_DIR")?;
    let chip_database = chip_database(cache, &database, &nextpnr, progress)?;
    let mut timing = None;
    for clock_divisor in CLOCK_DIVISORS {
        let netlist = synthesize(&directory, clock_divisor, progress)?;
        let routed = place_and_route(&directory, &chip_database, &netlist, progress)?;
        if routed.timing.is_met() {
            let frames = generate_frames(&directory, &database, &routed.assembly, progress)?;
            let path = write_bitstream(&directory, &database, &frames, progress)?;
            return Ok(Bitstream {
                path,
                clock_divisor,
            });
        }
        tracing::info!(
            achieved_megahertz = routed.timing.achieved,
            constraint_megahertz = routed.timing.constraint,
            "the circuit misses its system clock"
        );
        timing = Some(routed.timing);
    }
    let timing = timing.expect("at least one clock divisor");
    Err(ToolchainError::Timing {
        achieved_megahertz: timing.achieved,
        constraint_megahertz: timing.constraint,
    })
}

fn synthesize(
    directory: &Path,
    clock_divisor: u32,
    progress: &dyn Fn(Stage),
) -> Result<PathBuf, ToolchainError> {
    let netlist = directory.join("circuit.json");
    let script = directory.join("synthesis.ys");
    let sources = rtl::sources()
        .iter()
        .map(|source| yosys_path(&directory.join(source)))
        .collect::<Result<Vec<_>, _>>()?
        .join(" ");
    // A slice's flip-flops share one enable and reset, so enables and resets of fewer than
    // CONTROL_SET_FLIP_FLOPS flip-flops, such as each node's own, become LUT logic instead of
    // claiming a slice each.
    let synth_xilinx = "synth_xilinx -flatten -abc9 -nosrl -arch xc7 -top basys3 -run";
    let flip_flops = "-cell $_DFFE_?P?P_ 01 -cell $_SDFFE_?P?P_ 01 -cell $_DLATCH_?P?_ 01";
    fs::write(
        &script,
        format!(
            "read_verilog -sv {sources}
chparam -set CLOCK_DIVISOR {clock_divisor} basys3
{synth_xilinx} :map_ffs
dfflegalize {flip_flops} -mince {CONTROL_SET_FLIP_FLOPS} -minsrst {CONTROL_SET_FLIP_FLOPS}
{synth_xilinx} map_ffs:
write_json {}
",
            yosys_path(&netlist)?
        ),
    )?;
    progress(Stage::SynthesizeLogic);
    run(
        Command::new("yosys").arg("-s").arg(script),
        directory,
        "synthesis",
        BUILD_TIMEOUT,
    )?;
    Ok(netlist)
}

fn chip_database(
    cache: &Path,
    database: &Path,
    nextpnr: &Path,
    progress: &dyn Fn(Stage),
) -> Result<PathBuf, ToolchainError> {
    let chip_database = cache.join(format!("chipdb-{CHIP_DATABASE_DEVICE}.bin"));
    let lock_path = cache.join("chipdb.lock");
    let lock = fs::OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .open(&lock_path)?;
    progress(Stage::WaitForDeviceDatabase);
    cache::lock(&lock, Instant::now() + LOCK_TIMEOUT).map_err(|source| ToolchainError::Lock {
        path: lock_path,
        source,
    })?;
    if !chip_database.is_file() {
        build_chip_database(cache, database, nextpnr, &chip_database, progress)?;
    }
    lock.unlock()?;
    Ok(chip_database)
}

fn build_chip_database(
    cache: &Path,
    database: &Path,
    nextpnr: &Path,
    chip_database: &Path,
    progress: &dyn Fn(Stage),
) -> Result<(), ToolchainError> {
    let intermediate = cache.join(format!("chipdb-{CHIP_DATABASE_DEVICE}.bba"));
    let partial = cache.join(format!("chipdb-{CHIP_DATABASE_DEVICE}.bin.partial"));
    progress(Stage::BuildDeviceDatabase);
    run(
        Command::new("uv")
            .args(["run", "--no-project", "--python", "python3"])
            .arg(nextpnr.join("share/nextpnr/himbaechel/uarch/xilinx/gen/xilinx_gen.py"))
            .arg("--xray")
            .arg(database)
            .args(["--device", CHIP_DATABASE_DEVICE, "--bba"])
            .arg(&intermediate),
        cache,
        "chipdb-generate",
        BUILD_TIMEOUT,
    )?;
    progress(Stage::AssembleDeviceDatabase);
    run(
        Command::new("bbasm")
            .arg("-l")
            .arg(&intermediate)
            .arg(&partial),
        cache,
        "chipdb-assemble",
        ASSEMBLY_TIMEOUT,
    )?;
    fs::rename(partial, chip_database)?;
    fs::remove_file(intermediate)?;
    Ok(())
}

struct Routed {
    assembly: PathBuf,
    timing: ClockTiming,
}

#[derive(Deserialize)]
struct TimingReport {
    fmax: HashMap<String, ClockTiming>,
}

/// Frequencies in megahertz
#[derive(Clone, Copy, Deserialize)]
struct ClockTiming {
    achieved: f64,
    constraint: f64,
}

impl ClockTiming {
    fn headroom(self) -> f64 {
        self.achieved / self.constraint
    }

    fn is_met(self) -> bool {
        self.headroom() >= 1.0
    }
}

fn place_and_route(
    directory: &Path,
    chip_database: &Path,
    netlist: &Path,
    progress: &dyn Fn(Stage),
) -> Result<Routed, ToolchainError> {
    let constraints = directory
        .join(rtl::LIBRARY_DIRECTORY)
        .join("board")
        .join("basys3.xdc");
    let assembly = directory.join("circuit.fasm");
    let timing_report = directory.join("timing.json");
    progress(Stage::PlaceAndRoute);
    run(
        Command::new("nextpnr-himbaechel")
            .arg("--chipdb")
            .arg(chip_database)
            .args(["--device", DEVICE, "-o"])
            .arg(format!("xdc={}", constraints.display()))
            .arg("--json")
            .arg(netlist)
            .arg("-o")
            .arg(format!("fasm={}", assembly.display()))
            // nextpnr derives every clock's constraint from the board clock in the constraints file
            .arg("--report")
            .arg(&timing_report)
            .arg("--timing-allow-fail"),
        directory,
        "place-route",
        BUILD_TIMEOUT,
    )?;
    let report: TimingReport =
        serde_json::from_slice(&fs::read(&timing_report)?).map_err(|source| {
            ToolchainError::Report {
                path: timing_report,
                source,
            }
        })?;
    let timing = report
        .fmax
        .into_values()
        .min_by(|first, second| first.headroom().total_cmp(&second.headroom()))
        .ok_or(ToolchainError::Unclocked)?;
    Ok(Routed { assembly, timing })
}

fn generate_frames(
    directory: &Path,
    database: &Path,
    assembly: &Path,
    progress: &dyn Fn(Stage),
) -> Result<PathBuf, ToolchainError> {
    let frames = directory.join("circuit.frames");
    progress(Stage::GenerateConfigurationFrames);
    run(
        Command::new("fasm2frames")
            .arg("--db-root")
            .arg(database)
            .args(["--part", DEVICE])
            .arg(assembly)
            .arg(&frames),
        directory,
        "frames",
        CONVERSION_TIMEOUT,
    )?;
    Ok(frames)
}

fn write_bitstream(
    directory: &Path,
    database: &Path,
    frames: &Path,
    progress: &dyn Fn(Stage),
) -> Result<PathBuf, ToolchainError> {
    let bitstream = directory.join("circuit.bit");
    progress(Stage::WriteBitstream);
    run(
        Command::new("xc7frames2bit")
            .arg("--part_file")
            .arg(database.join(DEVICE).join("part.yaml"))
            .args(["--part_name", DEVICE, "--frm_file"])
            .arg(frames)
            .arg("--output_file")
            .arg(&bitstream),
        directory,
        "bitstream",
        CONVERSION_TIMEOUT,
    )?;
    Ok(bitstream)
}

pub(super) fn program(bitstream: &Path) -> Result<(), ToolchainError> {
    let bitstream = bitstream.canonicalize()?;
    let mut command = Command::new("openFPGALoader");
    command.args(["-b", "basys3", "--write-sram"]);
    if let Some(serial) = env::var_os("MCHPRS_FPGA_SERIAL").filter(|serial| !serial.is_empty()) {
        command.arg("--ftdi-serial").arg(serial);
    }
    run(
        command.arg(&bitstream),
        parent(&bitstream)?,
        "program",
        PROGRAM_TIMEOUT,
    )
}

fn yosys_path(path: &Path) -> Result<String, ToolchainError> {
    let text = path
        .to_str()
        .filter(|text| !text.contains(['\r', '\n']))
        .ok_or_else(|| ToolchainError::YosysPath(path.to_owned()))?;
    Ok(format!(
        "\"{}\"",
        text.replace('\\', "\\\\").replace('"', "\\\"")
    ))
}

fn environment_path(name: &'static str) -> Result<PathBuf, ToolchainError> {
    env::var_os(name)
        .filter(|value| !value.is_empty())
        .map(PathBuf::from)
        .ok_or(ToolchainError::Environment(name))
}

fn parent(path: &Path) -> Result<&Path, ToolchainError> {
    path.parent()
        .ok_or_else(|| ToolchainError::Orphan(path.to_owned()))
}

fn run(
    command: &mut Command,
    directory: &Path,
    stage: &'static str,
    timeout: Duration,
) -> Result<(), ToolchainError> {
    let stdout = directory.join(format!("{stage}.stdout.log"));
    let stderr = directory.join(format!("{stage}.stderr.log"));
    command
        .current_dir(directory)
        .stdin(Stdio::null())
        .stdout(File::create(&stdout)?)
        .stderr(File::create(&stderr)?);
    subprocess::run(command, Instant::now() + timeout).map_err(|error| match error {
        SubprocessError::Start { program, source } => ToolchainError::MissingTool {
            stage,
            program,
            source,
        },
        source => ToolchainError::Stage {
            stage,
            stdout,
            stderr,
            source,
        },
    })
}
