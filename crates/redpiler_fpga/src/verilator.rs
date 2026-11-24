//! After replying, Verilator clocks the circuit until it pauses or reaches
//! the `RUN_ON_CLOCKS` limit in `rtl/sim/top.sv`.

use crate::build_directory;
use crate::link::ByteLink;
use crate::process;
use crate::progress::Progress;
use crate::rtl::{self, CircuitSource, Sources};
use crate::tick;
use anyhow::Context;
use mchprs_simulation::backend::CompileError;
use std::io::{self, Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::time::Duration;
use tempfile::TempDir;

const OUTPUT: &str = "verilated";
const EXECUTABLE: &str = "simulation";
const BUILD_TIMEOUT: Duration = Duration::from_secs(30 * 60);
/// Yosys writes all of `state` on one line of `steps.v`, whatever its length.
const LINE_TOKEN_LIMIT: &str = "2147483647";

/// `sim/top.sv` clocks the circuit three times for every two clocks of the 100 MHz board clock.
pub const CIRCUIT_CLOCK_HERTZ: f64 = 150e6;

#[derive(Debug)]
pub struct VerilatorExecutable {
    path: PathBuf,
    directory: TempDir,
}

impl VerilatorExecutable {
    pub fn build(
        circuit: &CircuitSource,
        progress: &mut Progress,
    ) -> Result<VerilatorExecutable, CompileError> {
        let mut directory = build_directory::create()?;
        let verilated = verilate(circuit, directory.path(), progress);
        verilated.map_err(|error| build_directory::keep_if_a_tool_failed(&mut directory, error))?;
        Ok(VerilatorExecutable {
            path: directory.path().join(OUTPUT).join(EXECUTABLE),
            directory,
        })
    }

    pub fn start(self) -> io::Result<VerilatorLink> {
        let mut process = Command::new(&self.path)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()?;
        Ok(VerilatorLink {
            requests: process.stdin.take().expect("stdin is piped"),
            answers: process.stdout.take().expect("stdout is piped"),
            process,
            _directory: self.directory,
        })
    }
}

fn verilate(
    circuit: &CircuitSource,
    directory: &Path,
    progress: &mut Progress,
) -> Result<(), CompileError> {
    let sources = Sources::write(directory, circuit, rtl::SIMULATION);
    let sources = sources
        .context("could not write the sources")
        .map_err(CompileError::Failed)?;
    progress.start("Composing tick")?;
    let sources = tick::compose(sources, progress)?;
    progress.start("Building simulation")?;
    let mut verilator = Command::new("verilator");
    verilator
        .current_dir(directory)
        .args(["--binary", "-j", "0", "-Wno-PROCASSINIT"])
        .args(["--preproc-token-limit", LINE_TOKEN_LIMIT])
        .args(["--top-module", "top", "-Mdir", OUTPUT, "-o", EXECUTABLE])
        .args(sources.files());
    let log = directory.join("verilator.log");
    Ok(process::run(&mut verilator, &log, BUILD_TIMEOUT, progress)?)
}

#[derive(Debug)]
pub struct VerilatorLink {
    process: Child,
    requests: ChildStdin,
    answers: ChildStdout,
    _directory: TempDir,
}

impl ByteLink for VerilatorLink {
    fn write(&mut self, bytes: &[u8]) -> io::Result<()> {
        self.requests.write_all(bytes)
    }

    fn read(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
        match self.answers.read(buffer)? {
            0 => Err(io::ErrorKind::UnexpectedEof.into()),
            count => Ok(count),
        }
    }
}

impl Drop for VerilatorLink {
    fn drop(&mut self) {
        let _ = self.process.kill();
        let _ = self.process.wait();
    }
}
