use crate::progress::Progress;
use mchprs_simulation::backend::{Cancelled, CompileError};
use std::fs::File;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::{Duration, Instant};
use std::{io, thread};

const POLL_INTERVAL: Duration = Duration::from_millis(20);

#[derive(Debug, thiserror::Error)]
pub enum ToolError {
    #[error("could not start {program}, which has to be on the PATH")]
    Start {
        program: String,
        #[source]
        source: io::Error,
    },
    #[error("could not run {program} with the log {}", log.display())]
    Io {
        program: String,
        log: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("{program} failed with {status}: see {}", log.display())]
    Failed {
        program: String,
        status: ExitStatus,
        log: PathBuf,
    },
    #[error("{program} did not finish within {timeout:?}: see {}", log.display())]
    TimedOut {
        program: String,
        timeout: Duration,
        log: PathBuf,
    },
    #[error(transparent)]
    Cancelled(#[from] Cancelled),
}

impl From<ToolError> for CompileError {
    fn from(error: ToolError) -> CompileError {
        match error {
            ToolError::Cancelled(cancelled) => cancelled.into(),
            error => CompileError::Failed(error.into()),
        }
    }
}

pub fn run(
    command: &mut Command,
    log: &Path,
    timeout: Duration,
    progress: &Progress,
) -> Result<(), ToolError> {
    let program = command.get_program().to_string_lossy().into_owned();
    let failed_io = |source| ToolError::Io {
        program: program.clone(),
        log: log.to_owned(),
        source,
    };
    let output = File::create(log).map_err(failed_io)?;
    command
        .stdin(Stdio::null())
        .stdout(output.try_clone().map_err(failed_io)?)
        .stderr(output);
    isolate_process_group(command);
    let mut tool = command.spawn().map_err(|source| ToolError::Start {
        program: program.clone(),
        source,
    })?;
    let started = Instant::now();
    loop {
        let stopped = match tool.try_wait().map_err(failed_io)? {
            Some(status) if status.success() => return Ok(()),
            Some(status) => {
                return Err(ToolError::Failed {
                    program,
                    status,
                    log: log.to_owned(),
                });
            }
            None if progress.is_cancelled() => Cancelled.into(),
            None if started.elapsed() > timeout => ToolError::TimedOut {
                program,
                timeout,
                log: log.to_owned(),
            },
            None => {
                thread::sleep(POLL_INTERVAL);
                continue;
            }
        };
        kill(&mut tool);
        // A killed tool that is stuck in the kernel exits late or never.
        thread::spawn(move || tool.wait());
        return Err(stopped);
    }
}

// Linux ties the parent-death signal to the spawning thread. Descendants do not inherit it.
#[cfg(target_os = "linux")]
fn isolate_process_group(command: &mut Command) {
    use nix::sys::prctl::set_pdeathsig;
    use nix::sys::signal::Signal;
    use std::os::unix::process::CommandExt;

    command.process_group(0);
    // SAFETY: between `fork` and `exec` the closure makes one system call and allocates nothing.
    unsafe { command.pre_exec(|| Ok(set_pdeathsig(Signal::SIGKILL)?)) };
}

#[cfg(target_os = "linux")]
fn kill(tool: &mut Child) {
    use nix::sys::signal::{killpg, Signal};
    use nix::unistd::Pid;

    if let Ok(group) = i32::try_from(tool.id()) {
        let _ = killpg(Pid::from_raw(group), Signal::SIGKILL);
    }
}

#[cfg(not(target_os = "linux"))]
fn isolate_process_group(_: &mut Command) {}

#[cfg(not(target_os = "linux"))]
fn kill(tool: &mut Child) {
    let _ = tool.kill();
}
