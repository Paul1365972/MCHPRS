use crate::rtl::RTL;
use std::env;
use std::fs::{self, File, TryLockError};
use std::io;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::thread;
use std::time::{Duration, Instant};

const DEVICE: &str = "xc7a35tcpg236-1";
const POLL_INTERVAL: Duration = Duration::from_millis(25);
const BUILD_TIMEOUT: Duration = Duration::from_secs(30 * 60);
const ASSEMBLY_TIMEOUT: Duration = Duration::from_secs(10 * 60);
const CONVERSION_TIMEOUT: Duration = Duration::from_secs(5 * 60);
const PROGRAM_TIMEOUT: Duration = Duration::from_secs(60);
const LOCK_TIMEOUT: Duration = Duration::from_secs(45 * 60);

pub(super) fn build(directory: &Path, progress: &dyn Fn(&'static str)) -> io::Result<PathBuf> {
    let directory = directory.canonicalize()?;
    let output_root = directory
        .parent()
        .ok_or_else(|| io::Error::other("missing build cache directory"))?;
    let circuit = directory.join("circuit.v").canonicalize()?;
    let mut sources: Vec<_> = RTL
        .find("**/*.v")
        .expect("valid Verilog glob")
        .map(|entry| directory.join("rtl").join(entry.path()))
        .collect();
    sources.sort();
    sources.push(circuit);
    let constraints = directory.join("rtl/basys3.xdc");
    let database = environment_path("PRJXRAY_DB_DIR")?.join("artix7");
    let nextpnr = environment_path("NEXTPNR_XILINX_DIR")?;
    let circuit = directory.join("circuit.json");
    let assembly = directory.join("circuit.fasm");
    let frames = directory.join("circuit.frames");
    let bitstream = directory.join("circuit.bit");
    let script = directory.join("synthesis.ys");
    let inputs = sources
        .iter()
        .map(|source| yosys_path(source))
        .collect::<io::Result<Vec<_>>>()?
        .join(" ");
    fs::write(
        &script,
        format!(
            "read_verilog -sv {inputs}\nsynth_xilinx -flatten -abc9 -arch xc7 -top basys3\nwrite_json {}\n",
            yosys_path(&circuit)?
        ),
    )?;
    progress("Synthesizing logic");
    run(
        Command::new("yosys").arg("-s").arg(script),
        &directory,
        "synthesis",
        BUILD_TIMEOUT,
    )?;

    let chip_database = output_root.join("chipdb-xc7a50t.bin");
    {
        let cache_lock = fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(output_root.join("chipdb.lock"))?;
        progress("Waiting for device database");
        wait_for_lock(&cache_lock, LOCK_TIMEOUT).map_err(|error| {
            io::Error::new(
                error.kind(),
                format!(
                    "FPGA chip database lock {}: {error}",
                    output_root.join("chipdb.lock").display()
                ),
            )
        })?;
        if !chip_database.is_file() {
            let intermediate = output_root.join("chipdb-xc7a50t.bba");
            let pending = output_root.join("chipdb-xc7a50t.bin.partial");
            progress("Building device database");
            run(
                Command::new("uv")
                    .args(["run", "--no-project", "--python", "python3"])
                    .arg(nextpnr.join("share/nextpnr/himbaechel/uarch/xilinx/gen/xilinx_gen.py"))
                    .arg("--xray")
                    .arg(&database)
                    .args(["--device", "xc7a50t", "--bba"])
                    .arg(&intermediate),
                output_root,
                "chipdb-generate",
                BUILD_TIMEOUT,
            )?;
            progress("Assembling device database");
            run(
                Command::new("bbasm")
                    .arg("-l")
                    .arg(&intermediate)
                    .arg(&pending),
                output_root,
                "chipdb-assemble",
                ASSEMBLY_TIMEOUT,
            )?;
            fs::rename(pending, &chip_database)?;
            fs::remove_file(intermediate)?;
        }
    }

    progress("Placing and routing");
    run(
        Command::new("nextpnr-himbaechel")
            .arg("--chipdb")
            .arg(chip_database)
            .args(["--device", DEVICE, "-o"])
            .arg(format!("xdc={}", constraints.display()))
            .arg("--json")
            .arg(circuit)
            .arg("-o")
            .arg(format!("fasm={}", assembly.display()))
            .args(["--freq", "100"]),
        &directory,
        "place-route",
        BUILD_TIMEOUT,
    )?;
    progress("Generating configuration frames");
    run(
        Command::new("fasm2frames")
            .arg("--db-root")
            .arg(&database)
            .args(["--part", DEVICE])
            .arg(assembly)
            .arg(&frames),
        &directory,
        "frames",
        CONVERSION_TIMEOUT,
    )?;
    progress("Writing bitstream");
    run(
        Command::new("xc7frames2bit")
            .arg("--part_file")
            .arg(database.join(DEVICE).join("part.yaml"))
            .args(["--part_name", DEVICE, "--frm_file"])
            .arg(frames)
            .arg("--output_file")
            .arg(&bitstream),
        &directory,
        "bitstream",
        CONVERSION_TIMEOUT,
    )?;
    Ok(bitstream)
}

pub(super) fn program(bitstream: &Path) -> io::Result<()> {
    let bitstream = bitstream.canonicalize()?;
    let directory = bitstream
        .parent()
        .ok_or_else(|| io::Error::other("missing bitstream directory"))?
        .to_path_buf();
    let mut command = Command::new("openFPGALoader");
    command.args(["-b", "basys3", "--write-sram"]);
    if let Some(serial) = env::var_os("MCHPRS_FPGA_SERIAL").filter(|serial| !serial.is_empty()) {
        command.arg("--ftdi-serial").arg(serial);
    }
    run(
        command.arg(&bitstream),
        &directory,
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
    let result = (|| {
        let mut child = command
            .current_dir(directory)
            .stdin(Stdio::null())
            .stdout(File::create(&stdout)?)
            .stderr(File::create(&stderr)?)
            .spawn()
            .map_err(|error| {
                io::Error::new(
                    error.kind(),
                    format!(
                        "could not start {}: {error}; enter the FPGA Nix development shell",
                        command.get_program().to_string_lossy()
                    ),
                )
            })?;
        let status = wait_for_child(&mut child, timeout)?;
        if status.success() {
            Ok(())
        } else {
            Err(io::Error::other(format!("failed with {status}")))
        }
    })();
    result.map_err(|error| {
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

fn check_wait(started: Instant, timeout: Duration) -> io::Result<()> {
    if started.elapsed() >= timeout {
        Err(io::Error::new(
            io::ErrorKind::TimedOut,
            format!("timed out after {} seconds", timeout.as_secs_f64()),
        ))
    } else {
        Ok(())
    }
}

fn wait_for_child(child: &mut Child, timeout: Duration) -> io::Result<ExitStatus> {
    let started = Instant::now();
    loop {
        let result = check_wait(started, timeout).and_then(|()| child.try_wait());
        match result {
            Ok(Some(status)) => return Ok(status),
            Ok(None) => thread::sleep(POLL_INTERVAL.min(timeout.saturating_sub(started.elapsed()))),
            Err(error) => {
                let kill_error = child.kill().err();
                return match child.wait() {
                    Ok(_) => Err(error),
                    Err(wait_error) => Err(io::Error::new(
                        error.kind(),
                        format!(
                            "{error}; child cleanup failed: kill={kill_error:?}, wait={wait_error}"
                        ),
                    )),
                };
            }
        }
    }
}

fn wait_for_lock(file: &File, timeout: Duration) -> io::Result<()> {
    let started = Instant::now();
    loop {
        check_wait(started, timeout)?;
        match file.try_lock() {
            Ok(()) => return Ok(()),
            Err(TryLockError::WouldBlock) => {
                thread::sleep(POLL_INTERVAL.min(timeout.saturating_sub(started.elapsed())));
            }
            Err(TryLockError::Error(error)) => return Err(error),
        }
    }
}
