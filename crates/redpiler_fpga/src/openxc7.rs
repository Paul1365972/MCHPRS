use crate::board::{Board, BoardPort};
use crate::process;
use crate::progress::Progress;
use crate::rtl::{self, Sources};
use mchprs_simulation::backend::{Cancelled, CompileError};
use serde::Deserialize;
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Duration;
use std::{env, fs, io};

const SCRIPT: &str = "synthesis.ys";
const NETLIST: &str = "design.json";
const ASSEMBLY: &str = "design.fasm";
const REPORT: &str = "report.json";
const FRAMES: &str = "design.frames";
const BITSTREAM: &str = "design.bit";
const CIRCUIT_CLOCK: &str = "circuit_clock";
const CHIP_DATABASE_DIRECTORY: &str = "NEXTPNR_XILINX_CHIPDB_DIR";
const PRJXRAY_DATABASE_DIRECTORY: &str = "PRJXRAY_DB_DIR";
const LUTS: &str = "SLICE_LUTX";
const FLIP_FLOPS: &str = "SLICE_FFX";
// Measured on BatPU-1: 4, 8 and 16 give nearly the same number of LUTs.
const CONTROL_SET_FLIP_FLOPS: u32 = 4;
const SYNTHESIS_TIMEOUT: Duration = Duration::from_secs(30 * 60);
const ROUTING_TIMEOUT: Duration = Duration::from_secs(60 * 60);
const CONVERSION_TIMEOUT: Duration = Duration::from_secs(5 * 60);
const PROGRAMMER_TIMEOUT: Duration = Duration::from_secs(60);

#[derive(Debug, Clone, PartialEq)]
pub struct Bitstream {
    pub path: PathBuf,
    pub fmax_hertz: f64,
    pub luts: u32,
    pub flip_flops: u32,
    pub slices: usize,
}

#[derive(Debug, thiserror::Error)]
pub enum ToolchainError {
    #[error(transparent)]
    Tool(#[from] process::ToolError),
    #[error("could not access {}", path.display())]
    Io {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("the report {} is not what the tool is known to write", path.display())]
    Report {
        path: PathBuf,
        #[source]
        source: serde_json::Error,
    },
    #[error("the report names no clock {0}")]
    NoClock(&'static str),
    #[error("the clock {clock} of the board reaches {achieved:.1} of its {required:.1} MHz")]
    Timing {
        clock: String,
        achieved: f64,
        required: f64,
    },
    #[error("{0} is not set to the directory of the prjxray database")]
    NoDatabase(&'static str),
}

impl From<Cancelled> for ToolchainError {
    fn from(cancelled: Cancelled) -> ToolchainError {
        ToolchainError::Tool(cancelled.into())
    }
}

impl From<ToolchainError> for CompileError {
    fn from(error: ToolchainError) -> CompileError {
        match error {
            ToolchainError::Tool(error) => error.into(),
            error => CompileError::Failed(error.into()),
        }
    }
}

pub fn detect(
    board: &Board,
    port: &BoardPort,
    directory: &Path,
    progress: &Progress,
) -> Result<(), ToolchainError> {
    let mut detect = programmer(board, port, directory);
    detect.arg("--detect");
    let log = directory.join("detect.log");
    Ok(process::run(
        &mut detect,
        &log,
        PROGRAMMER_TIMEOUT,
        progress,
    )?)
}

pub fn build(
    sources: &Sources,
    board: &Board,
    progress: &mut Progress,
) -> Result<Bitstream, ToolchainError> {
    let directory = sources.directory.as_path();
    let tool = |program: &str| {
        let mut tool = Command::new(program);
        tool.current_dir(directory);
        tool
    };
    let log = |name: &str| directory.join(format!("{name}.log"));
    let chip_database = env::var_os(CHIP_DATABASE_DIRECTORY).map(|directory| {
        let chip_database = format!("chipdb-{}.bin", board.chip_database_device);
        Path::new(&directory).join(chip_database)
    });
    let database = env::var_os(PRJXRAY_DATABASE_DIRECTORY)
        .ok_or(ToolchainError::NoDatabase(PRJXRAY_DATABASE_DIRECTORY))?;
    let database = Path::new(&database).join(board.prjxray_family);
    fs::write(
        directory.join(SCRIPT),
        synthesis_script(&sources.files().collect::<Vec<_>>(), board),
    )
    .and_then(|()| rtl::write(directory, [board.constraints]))
    .map_err(|source| inaccessible(directory, source))?;

    progress.start("Synthesizing")?;
    let mut yosys = tool("yosys");
    yosys.args(["-s", SCRIPT]);
    process::run(&mut yosys, &log("synthesis"), SYNTHESIS_TIMEOUT, progress)?;

    progress.start("Placing and routing")?;
    let megahertz = f64::from(board.oscillator_hertz) / 1e6;
    let mut nextpnr = tool("nextpnr-himbaechel");
    if let Some(chip_database) = chip_database {
        nextpnr.arg("--chipdb").arg(chip_database);
    }
    nextpnr
        .args(["--device", board.part, "--json", NETLIST])
        .args(["-o", &format!("xdc={}", board.constraints.path)])
        .args(["-o", &format!("fasm={ASSEMBLY}"), "--report", REPORT])
        // The circuit clock may miss the frequency: the host sets it to what the report says
        // it reaches.
        .args(["--freq", &megahertz.to_string(), "--timing-allow-fail"]);
    process::run(
        &mut nextpnr,
        &log("place-and-route"),
        ROUTING_TIMEOUT,
        progress,
    )?;
    let report = Report::read(&directory.join(REPORT))?;
    let fmax_hertz = report.circuit_fmax_hertz()?;
    let slices = used_slices(&read(&directory.join(ASSEMBLY))?);

    progress.start("Writing bitstream")?;
    let mut frames = tool("fasm2frames");
    frames.arg("--db-root").arg(&database);
    frames.args(["--part", board.part, ASSEMBLY, FRAMES]);
    process::run(&mut frames, &log("frames"), CONVERSION_TIMEOUT, progress)?;
    let mut bitstream = tool("xc7frames2bit");
    bitstream
        .arg("--part_file")
        .arg(database.join(board.part).join("part.yaml"))
        .args(["--part_name", board.part, "--frm_file", FRAMES])
        .args(["--output_file", BITSTREAM]);
    process::run(
        &mut bitstream,
        &log("bitstream"),
        CONVERSION_TIMEOUT,
        progress,
    )?;
    Ok(Bitstream {
        path: directory.join(BITSTREAM),
        fmax_hertz,
        luts: report.used(LUTS),
        flip_flops: report.used(FLIP_FLOPS),
        slices,
    })
}

pub fn program(
    bitstream: &Bitstream,
    board: &Board,
    port: &BoardPort,
    progress: &Progress,
) -> Result<(), ToolchainError> {
    let directory = bitstream.path.parent().unwrap_or(Path::new("."));
    let mut program = programmer(board, port, directory);
    program.arg(&bitstream.path);
    let log = directory.join("program.log");
    Ok(process::run(
        &mut program,
        &log,
        PROGRAMMER_TIMEOUT,
        progress,
    )?)
}

fn synthesis_script(files: &[&str], board: &Board) -> String {
    let synthesize = format!(
        "synth_xilinx -flatten -abc9 -nosrl -nodsp -top {}",
        board.top_module
    );
    format!(
        "read_verilog -sv {}\n\
         # nextpnr times no path through a DSP block (-nodsp), and its shift register cells read\n\
         # back as zeros (-nosrl).\n\
         {synthesize} -run :map_ffs\n\
         # The flip-flops of a slice share one enable and one reset. A node enables its own few,\n\
         # which would take a slice per node: below {flip_flops} flip-flops an enable or reset\n\
         # becomes logic.\n\
         dfflegalize -cell $_DFFE_?P?P_ 01 -cell $_SDFFE_?P?P_ 01 -cell $_DLATCH_?P?_ 01 \
         -mince {flip_flops} -minsrst {flip_flops}\n\
         {synthesize} -run map_ffs:\n\
         write_json {NETLIST}\n",
        files.join(" "),
        flip_flops = CONTROL_SET_FLIP_FLOPS,
    )
}

fn programmer(board: &Board, port: &BoardPort, directory: &Path) -> Command {
    let mut programmer = Command::new("openFPGALoader");
    programmer.current_dir(directory);
    let board = board.openfpgaloader_board;
    programmer.args(["-b", board, "--ftdi-serial", &port.usb_serial_number]);
    programmer
}

fn read(path: &Path) -> Result<String, ToolchainError> {
    fs::read_to_string(path).map_err(|source| inaccessible(path, source))
}

fn inaccessible(path: &Path, source: io::Error) -> ToolchainError {
    ToolchainError::Io {
        path: path.to_owned(),
        source,
    }
}

#[derive(Debug, Deserialize)]
struct Report {
    fmax: BTreeMap<String, ClockMegahertz>,
    utilization: BTreeMap<String, Utilization>,
}

#[derive(Debug, Deserialize)]
struct ClockMegahertz {
    achieved: f64,
    constraint: f64,
}

#[derive(Debug, Deserialize)]
struct Utilization {
    used: u32,
}

impl Report {
    fn read(path: &Path) -> Result<Report, ToolchainError> {
        serde_json::from_str(&read(path)?).map_err(|source| ToolchainError::Report {
            path: path.to_owned(),
            source,
        })
    }

    fn circuit_fmax_hertz(&self) -> Result<f64, ToolchainError> {
        let slow = |(_, clock): &(&String, &ClockMegahertz)| clock.achieved < clock.constraint;
        let mut board_clocks = self.fmax.iter().filter(|(net, _)| *net != CIRCUIT_CLOCK);
        if let Some((net, clock)) = board_clocks.find(slow) {
            return Err(ToolchainError::Timing {
                clock: net.clone(),
                achieved: clock.achieved,
                required: clock.constraint,
            });
        }
        let circuit = self.fmax.get(CIRCUIT_CLOCK);
        let circuit = circuit.ok_or(ToolchainError::NoClock(CIRCUIT_CLOCK))?;
        Ok(circuit.achieved * 1e6)
    }

    fn used(&self, cells: &str) -> u32 {
        self.utilization.get(cells).map_or(0, |cells| cells.used)
    }
}

fn used_slices(assembly: &str) -> usize {
    let slices: BTreeSet<&str> = assembly
        .lines()
        .filter_map(|line| {
            let (tile, features) = line.split_once('.')?;
            let (site, feature) = features.split_once('.')?;
            let element = feature.get(1..)?;
            let is_logic = feature.starts_with("CARRY4")
                || ["LUT", "FF", "5FF"]
                    .iter()
                    .any(|kind| element.starts_with(kind));
            let is_used = site.starts_with("SLICE") && is_logic;
            is_used.then(|| &line[..tile.len() + 1 + site.len()])
        })
        .collect();
    slices.len()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_slice_counts_once_and_only_with_logic() {
        let assembly = "\
            CLBLL_L_X2Y3.SLICEL_X0.ALUT.INIT[63:0] = 64'b1\n\
            CLBLL_L_X2Y3.SLICEL_X0.AFF.ZINI\n\
            CLBLL_L_X2Y3.SLICEL_X1.B5FF.ZRST\n\
            CLBLM_R_X3Y5.SLICEM_X0.CARRY4.ACY0\n\
            CLBLM_R_X3Y5.SLICEL_X1.NOCLKINV\n\
            CLBLM_R_X3Y5.SLICEL_X1.CEUSEDMUX\n\
            INT_L_X2Y3.NL1BEG1.LOGIC_OUTS_L0\n\
            CMT_TOP_R_UPPER_T_X106Y113.PLLE2_ADV.IN_USE\n";
        assert_eq!(used_slices(assembly), 3);
    }
}
