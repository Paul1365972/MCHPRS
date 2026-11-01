use crate::cache;
use crate::rtl;
use crate::subprocess::{self, SubprocessError};
use std::env;
use std::fs::{self, File};
use std::io;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

const DEVICE: &str = "xc7a35tcpg236-1";
// The XC7A35T is an XC7A50T die, and nextpnr only has a chip database for the latter.
const CHIP_DATABASE_DEVICE: &str = "xc7a50t";
const BUILD_TIMEOUT: Duration = Duration::from_secs(30 * 60);
const ASSEMBLY_TIMEOUT: Duration = Duration::from_secs(10 * 60);
const CONVERSION_TIMEOUT: Duration = Duration::from_secs(5 * 60);
const PROGRAM_TIMEOUT: Duration = Duration::from_secs(60);
const LOCK_TIMEOUT: Duration = Duration::from_secs(45 * 60);

pub(super) fn build(directory: &Path, progress: &dyn Fn(&'static str)) -> io::Result<PathBuf> {
    let directory = directory.canonicalize()?;
    let cache = directory
        .parent()
        .ok_or_else(|| io::Error::other("missing build cache directory"))?;
    let database = environment_path("PRJXRAY_DB_DIR")?.join("artix7");
    let nextpnr = environment_path("NEXTPNR_XILINX_DIR")?;
    let netlist = synthesize(&directory, progress)?;
    let chip_database = chip_database(cache, &database, &nextpnr, progress)?;
    let assembly = place_and_route(&directory, &chip_database, &netlist, progress)?;
    let frames = generate_frames(&directory, &database, &assembly, progress)?;
    write_bitstream(&directory, &database, &frames, progress)
}

fn synthesize(directory: &Path, progress: &dyn Fn(&'static str)) -> io::Result<PathBuf> {
    let netlist = directory.join("circuit.json");
    let script = directory.join("synthesis.ys");
    let sources = rtl::sources()
        .iter()
        .map(|source| yosys_path(&directory.join(source)))
        .collect::<io::Result<Vec<_>>>()?
        .join(" ");
    fs::write(
        &script,
        format!(
            "read_verilog -sv {sources}\nsynth_xilinx -flatten -abc9 -arch xc7 -top basys3\nwrite_json {}\n",
            yosys_path(&netlist)?
        ),
    )?;
    progress("Synthesizing logic");
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
    progress: &dyn Fn(&'static str),
) -> io::Result<PathBuf> {
    let chip_database = cache.join(format!("chipdb-{CHIP_DATABASE_DEVICE}.bin"));
    let lock_path = cache.join("chipdb.lock");
    let lock = fs::OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .open(&lock_path)?;
    progress("Waiting for device database");
    cache::lock(&lock, Instant::now() + LOCK_TIMEOUT).map_err(|error| {
        io::Error::new(
            error.kind(),
            format!("FPGA chip database lock {}: {error}", lock_path.display()),
        )
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
    progress: &dyn Fn(&'static str),
) -> io::Result<()> {
    let intermediate = cache.join(format!("chipdb-{CHIP_DATABASE_DEVICE}.bba"));
    let partial = cache.join(format!("chipdb-{CHIP_DATABASE_DEVICE}.bin.partial"));
    progress("Building device database");
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
    progress("Assembling device database");
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
    fs::remove_file(intermediate)
}

fn place_and_route(
    directory: &Path,
    chip_database: &Path,
    netlist: &Path,
    progress: &dyn Fn(&'static str),
) -> io::Result<PathBuf> {
    let constraints = directory.join(rtl::LIBRARY_DIRECTORY).join("basys3.xdc");
    let assembly = directory.join("circuit.fasm");
    progress("Placing and routing");
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
            .args(["--freq", "100"]),
        directory,
        "place-route",
        BUILD_TIMEOUT,
    )?;
    Ok(assembly)
}

fn generate_frames(
    directory: &Path,
    database: &Path,
    assembly: &Path,
    progress: &dyn Fn(&'static str),
) -> io::Result<PathBuf> {
    let frames = directory.join("circuit.frames");
    progress("Generating configuration frames");
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
    progress: &dyn Fn(&'static str),
) -> io::Result<PathBuf> {
    let bitstream = directory.join("circuit.bit");
    progress("Writing bitstream");
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

pub(super) fn program(bitstream: &Path) -> io::Result<()> {
    let bitstream = bitstream.canonicalize()?;
    let directory = bitstream
        .parent()
        .ok_or_else(|| io::Error::other("missing bitstream directory"))?;
    let mut command = Command::new("openFPGALoader");
    command.args(["-b", "basys3", "--write-sram"]);
    if let Some(serial) = env::var_os("MCHPRS_FPGA_SERIAL").filter(|serial| !serial.is_empty()) {
        command.arg("--ftdi-serial").arg(serial);
    }
    run(
        command.arg(&bitstream),
        directory,
        "program",
        PROGRAM_TIMEOUT,
    )
}

fn yosys_path(path: &Path) -> io::Result<String> {
    let path = path.to_str().filter(|value| !value.contains(['\r', '\n']));
    let path = path.ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            "Yosys paths must be UTF-8 without line breaks",
        )
    })?;
    Ok(format!(
        "\"{}\"",
        path.replace('\\', "\\\\").replace('"', "\\\"")
    ))
}

fn environment_path(name: &str) -> io::Result<PathBuf> {
    env::var_os(name)
        .filter(|value| !value.is_empty())
        .map(PathBuf::from)
        .ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::NotFound,
                format!("{name} is not set; enter an openXC7 development environment"),
            )
        })
}

fn run(command: &mut Command, directory: &Path, stage: &str, timeout: Duration) -> io::Result<()> {
    let stdout = directory.join(format!("{stage}.stdout.log"));
    let stderr = directory.join(format!("{stage}.stderr.log"));
    run_logged(command, directory, &stdout, &stderr, timeout).map_err(|error| {
        io::Error::new(
            error.kind(),
            format!(
                "FPGA {stage}: {error}; see {} and {}",
                stdout.display(),
                stderr.display()
            ),
        )
    })
}

fn run_logged(
    command: &mut Command,
    directory: &Path,
    stdout: &Path,
    stderr: &Path,
    timeout: Duration,
) -> io::Result<()> {
    command
        .current_dir(directory)
        .stdin(Stdio::null())
        .stdout(File::create(stdout)?)
        .stderr(File::create(stderr)?);
    subprocess::run(command, Instant::now() + timeout).map_err(|error| match error {
        SubprocessError::Start { .. } => {
            io::Error::other(format!("{error}; enter the FPGA Nix development shell"))
        }
        error => io::Error::other(error),
    })
}
