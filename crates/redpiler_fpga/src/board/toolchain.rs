use super::pll::{PllCountError, PllSetting};
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
/// The clock that runs the circuit, named after its net in the board top
const CIRCUIT_CLOCK: &str = "circuit_clock";
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
    #[error("the timing report has no {CIRCUIT_CLOCK} clock")]
    MissingCircuitClock,
    #[error("the {clock} clock reaches {achieved_megahertz:.1} MHz but must reach {required_megahertz:.1} MHz")]
    Timing {
        clock: String,
        achieved_megahertz: f64,
        required_megahertz: f64,
    },
    #[error("the {CIRCUIT_CLOCK} clock reaches only {0:.1} MHz, slower than the PLL can run")]
    SlowCircuitClock(f64),
    #[error(transparent)]
    PllCount(#[from] PllCountError),
    #[error("FPGA {stage} failed; see {} and {}", .stdout.display(), .stderr.display())]
    Stage {
        stage: &'static str,
        stdout: PathBuf,
        stderr: PathBuf,
        #[source]
        source: SubprocessError,
    },
}

/// What the board's seven-segment display shows
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Display {
    /// The tick rate, tick count or fault code, cycled by the left and right buttons
    SevenSegment,
    /// Nothing, which leaves out the display's logic
    Dark,
}

pub(super) struct Bitstream {
    pub(super) path: PathBuf,
    pub(super) circuit_clock_hertz: u32,
}

pub(super) fn build(
    directory: &Path,
    display: Display,
    progress: &dyn Fn(Stage),
) -> Result<Bitstream, ToolchainError> {
    let directory = directory.canonicalize()?;
    let cache = parent(&directory)?;
    let database = environment_path("PRJXRAY_DB_DIR")?.join("artix7");
    let nextpnr = environment_path("NEXTPNR_XILINX_DIR")?;
    let chip_database = chip_database(cache, &database, &nextpnr, progress)?;
    let netlist = synthesize(&directory, display, progress)?;
    let routed = place_and_route(&directory, &chip_database, &netlist, progress)?;
    let (setting, maximum_megahertz) = circuit_clock_setting(&routed.timing)?;
    let assembly = fs::read_to_string(&routed.assembly)?;
    fs::write(&routed.assembly, setting.apply(&assembly)?)?;
    let circuit_clock_hertz = setting.output_hertz() as u32;
    tracing::info!(
        circuit_clock_hertz,
        maximum_megahertz,
        ?setting,
        "chose the circuit clock"
    );
    let frames = generate_frames(&directory, &database, &routed.assembly, progress)?;
    let path = write_bitstream(&directory, &database, &frames, progress)?;
    Ok(Bitstream {
        path,
        circuit_clock_hertz,
    })
}

/// The PLL setting of the fastest circuit clock the routed circuit meets, with that maximum
/// frequency, provided every other clock meets its constraint
fn circuit_clock_setting(
    timing: &HashMap<String, ClockTiming>,
) -> Result<(PllSetting, f64), ToolchainError> {
    let mut maximum_megahertz = None;
    for (clock, timing) in timing {
        if clock == CIRCUIT_CLOCK {
            maximum_megahertz = Some(timing.achieved);
        } else if timing.achieved < timing.constraint {
            return Err(ToolchainError::Timing {
                clock: clock.clone(),
                achieved_megahertz: timing.achieved,
                required_megahertz: timing.constraint,
            });
        }
    }
    let maximum_megahertz = maximum_megahertz.ok_or(ToolchainError::MissingCircuitClock)?;
    let setting = PllSetting::fastest(maximum_megahertz * 1e6)
        .ok_or(ToolchainError::SlowCircuitClock(maximum_megahertz))?;
    Ok((setting, maximum_megahertz))
}

fn synthesize(
    directory: &Path,
    display: Display,
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
    let seven_segment_display = u8::from(display == Display::SevenSegment);
    fs::write(
        &script,
        format!(
            "read_verilog -sv {sources}
chparam -set SEVEN_SEGMENT_DISPLAY {seven_segment_display} basys3
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
    timing: HashMap<String, ClockTiming>,
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
    Ok(Routed {
        assembly,
        timing: report.fmax,
    })
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

#[cfg(test)]
mod tests {
    use super::*;

    fn timing(clocks: &[(&str, f64, f64)]) -> HashMap<String, ClockTiming> {
        clocks
            .iter()
            .map(|&(clock, achieved, constraint)| {
                let timing = ClockTiming {
                    achieved,
                    constraint,
                };
                (clock.to_owned(), timing)
            })
            .collect()
    }

    #[test]
    fn the_circuit_clock_runs_as_fast_as_it_meets() {
        let hertz = |megahertz| {
            let clocks = [(CIRCUIT_CLOCK, megahertz, 100.0), ("clock", 300.0, 100.0)];
            circuit_clock_setting(&timing(&clocks)).map(|(setting, _)| setting.output_hertz())
        };
        assert_eq!(hertz(108.6).unwrap().round(), 108_333_333.0);
        assert_eq!(hertz(100.0).unwrap(), 100e6);
        assert!(matches!(
            hertz(5.0),
            Err(ToolchainError::SlowCircuitClock(_))
        ));
    }

    #[test]
    fn other_clocks_must_meet_their_constraints() {
        let clocks = [(CIRCUIT_CLOCK, 120.0, 100.0), ("clock", 90.0, 100.0)];
        assert!(matches!(
            circuit_clock_setting(&timing(&clocks)),
            Err(ToolchainError::Timing { clock, .. }) if clock == "clock"
        ));
    }
}
