use crate::cache;
use crate::rtl;
use crate::stage::Stage;
use crate::subprocess::{self, SubprocessError};
use serde::Deserialize;
use std::collections::{BTreeSet, HashMap};
use std::env;
use std::fs::{self, File};
use std::io;
use std::ops::RangeInclusive;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

const DEVICE: &str = "xc7a35tcpg236-1";
// The XC7A35T shares the XC7A50T die, so nextpnr routes it with the XC7A50T chip database.
const CHIP_DATABASE_DEVICE: &str = "xc7a50t";
/// The clock that runs the circuit, named after its net in the board top
const CIRCUIT_CLOCK: &str = "circuit_clock";
/// The frequency of the PLL's oscillator, 16 times the board clock and the most the XC7A35T-1 allows
const PLL_OSCILLATOR_MEGAHERTZ: u32 = 1600;
/// The dividers of the PLL's oscillator that can drive the circuit clock: from 400 MHz, within the
/// global clock network's 464 MHz, to the most the counter's six-bit high and low times can count
const CIRCUIT_CLOCK_DIVIDERS: RangeInclusive<u32> = 4..=126;
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
    #[error("the routed design holds {0} PLLs instead of one")]
    PllCount(usize),
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
    pub(super) circuit_clock_hertz: u32,
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
    let netlist = synthesize(&directory, progress)?;
    let routed = place_and_route(&directory, &chip_database, &netlist, progress)?;
    let (divider, maximum_megahertz) = circuit_clock_divider(&routed.timing)?;
    let assembly = fs::read_to_string(&routed.assembly)?;
    fs::write(
        &routed.assembly,
        with_circuit_clock_divider(&assembly, divider)?,
    )?;
    let circuit_clock_hertz = PLL_OSCILLATOR_MEGAHERTZ * 1_000_000 / divider;
    tracing::info!(
        circuit_clock_hertz,
        maximum_megahertz,
        "chose the circuit clock"
    );
    let frames = generate_frames(&directory, &database, &routed.assembly, progress)?;
    let path = write_bitstream(&directory, &database, &frames, progress)?;
    Ok(Bitstream {
        path,
        circuit_clock_hertz,
    })
}

/// The divider of the PLL's oscillator that gives the fastest circuit clock the routed circuit
/// meets, with that maximum frequency, provided every other clock meets its constraint
fn circuit_clock_divider(
    timing: &HashMap<String, ClockTiming>,
) -> Result<(u32, f64), ToolchainError> {
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
    let oscillator_megahertz = f64::from(PLL_OSCILLATOR_MEGAHERTZ);
    let divider = ((oscillator_megahertz / maximum_megahertz).ceil() as u32)
        .max(*CIRCUIT_CLOCK_DIVIDERS.start());
    if !CIRCUIT_CLOCK_DIVIDERS.contains(&divider) {
        return Err(ToolchainError::Timing {
            clock: CIRCUIT_CLOCK.to_owned(),
            achieved_megahertz: maximum_megahertz,
            required_megahertz: oscillator_megahertz / f64::from(*CIRCUIT_CLOCK_DIVIDERS.end()),
        });
    }
    Ok((divider, maximum_megahertz))
}

/// The routed design's assembly with another divider for the PLL output that drives the circuit
/// clock. The output's counter splits each period into a high and a low time, and an odd divider
/// sets its edge bit for the extra half period.
fn with_circuit_clock_divider(assembly: &str, divider: u32) -> Result<String, ToolchainError> {
    const HIGH_TIME: &str = ".PLLE2_ADV.CLKOUT0_CLKOUT1_HIGH_TIME";
    const LOW_TIME: &str = ".PLLE2_ADV.CLKOUT0_CLKOUT1_LOW_TIME";
    const EDGE: &str = ".PLLE2_ADV.CLKOUT0_CLKOUT2_EDGE";
    const NO_COUNT: &str = ".PLLE2_ADV.CLKOUT0_CLKOUT2_NO_COUNT";
    let tiles: BTreeSet<&str> = assembly
        .lines()
        .filter_map(|line| line.split_once(HIGH_TIME).map(|(tile, _)| tile))
        .collect();
    let [tile] = tiles.iter().copied().collect::<Vec<_>>()[..] else {
        return Err(ToolchainError::PllCount(tiles.len()));
    };
    let high_time = divider / 2;
    let mut lines: Vec<String> = assembly
        .lines()
        .filter(|line| {
            ![HIGH_TIME, LOW_TIME, EDGE, NO_COUNT]
                .iter()
                .any(|field| line.contains(field))
        })
        .map(str::to_owned)
        .collect();
    lines.push(format!("{tile}{HIGH_TIME}[5:0] = 6'b{high_time:06b}"));
    lines.push(format!(
        "{tile}{LOW_TIME}[5:0] = 6'b{:06b}",
        divider - high_time
    ));
    if divider % 2 == 1 {
        lines.push(format!("{tile}{EDGE}[0]"));
    }
    lines.push(String::new());
    Ok(lines.join("\n"))
}

fn synthesize(directory: &Path, progress: &dyn Fn(Stage)) -> Result<PathBuf, ToolchainError> {
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

    /// nextpnr's PLL lines for a CLKOUT0_DIVIDE of 16, 19 and 32, around another PLL output's
    fn assembly(high_time: &str, low_time: &str, odd: bool) -> BTreeSet<String> {
        let pll = "CMT_TOP_L_UPPER_T_X106Y44.PLLE2_ADV";
        let mut lines = vec![
            format!("{pll}.IN_USE"),
            format!("{pll}.CLKFBOUT_CLKOUT1_HIGH_TIME[5:0] = 6'b001000"),
            format!("{pll}.CLKOUT0_CLKOUT1_OUTPUT_ENABLE[0]"),
            format!("{pll}.CLKOUT0_CLKOUT1_HIGH_TIME[5:0] = 6'b{high_time}"),
            format!("{pll}.CLKOUT0_CLKOUT1_LOW_TIME[5:0] = 6'b{low_time}"),
            format!("{pll}.CLKOUT0_CLKOUT2_DELAY_TIME[5:0] = 6'b000000"),
        ];
        if odd {
            lines.push(format!("{pll}.CLKOUT0_CLKOUT2_EDGE[0]"));
        }
        lines.into_iter().collect()
    }

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
    fn dividers_follow_the_encoding_of_nextpnr() {
        let sixteen = assembly("001000", "001000", false);
        let nineteen = assembly("001001", "001010", true);
        let thirty_two = assembly("010000", "010000", false);
        let divided = |lines: &BTreeSet<String>, divider| -> BTreeSet<String> {
            let text = Vec::from_iter(lines.iter().cloned()).join("\n");
            let divided = with_circuit_clock_divider(&text, divider).unwrap();
            divided.lines().map(str::to_owned).collect()
        };
        assert_eq!(divided(&sixteen, 19), nineteen);
        assert_eq!(divided(&nineteen, 16), sixteen);
        assert_eq!(divided(&sixteen, 32), thirty_two);
    }

    #[test]
    fn the_circuit_clock_takes_the_smallest_divider_it_meets() {
        let divider = |megahertz| {
            let clocks = [(CIRCUIT_CLOCK, megahertz, 100.0), ("clock", 300.0, 100.0)];
            circuit_clock_divider(&timing(&clocks))
                .map(|(divider, _)| divider)
                .ok()
        };
        assert_eq!(divider(87.9), Some(19));
        assert_eq!(divider(100.0), Some(16));
        assert_eq!(divider(116.8), Some(14));
        assert_eq!(divider(900.0), Some(4));
        assert_eq!(divider(12.0), None);
    }

    #[test]
    fn other_clocks_must_meet_their_constraints() {
        let clocks = [(CIRCUIT_CLOCK, 120.0, 100.0), ("clock", 90.0, 100.0)];
        assert!(matches!(
            circuit_clock_divider(&timing(&clocks)),
            Err(ToolchainError::Timing { clock, .. }) if clock == "clock"
        ));
    }
}
