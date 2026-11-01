use std::io;
use std::process::{Child, ChildStdin, ChildStdout, Command, ExitStatus};
use std::thread;
use std::time::{Duration, Instant};

const POLL_INTERVAL: Duration = Duration::from_millis(25);

#[derive(Debug, thiserror::Error)]
pub(crate) enum SubprocessError {
    #[error("could not start {program}")]
    Start {
        program: String,
        #[source]
        source: io::Error,
    },
    #[error("could not wait for {program}")]
    Wait {
        program: String,
        #[source]
        source: io::Error,
    },
    #[error("{program} timed out")]
    TimedOut { program: String },
    #[error("{program} failed with {status}")]
    Failed { program: String, status: ExitStatus },
}

/// A child process in its own process group, so that terminating it also ends its descendants.
pub(crate) struct Process {
    program: String,
    child: Child,
}

impl Process {
    pub(crate) fn spawn(command: &mut Command) -> Result<Self, SubprocessError> {
        #[cfg(unix)]
        {
            use std::os::unix::process::CommandExt;
            command.process_group(0);
        }
        let program = command.get_program().to_string_lossy().into_owned();
        match command.spawn() {
            Ok(child) => Ok(Self { program, child }),
            Err(source) => Err(SubprocessError::Start { program, source }),
        }
    }

    pub(crate) fn take_stdin(&mut self) -> Option<ChildStdin> {
        self.child.stdin.take()
    }

    pub(crate) fn take_stdout(&mut self) -> Option<ChildStdout> {
        self.child.stdout.take()
    }

    pub(crate) fn wait_until(&mut self, deadline: Instant) -> Result<(), SubprocessError> {
        loop {
            let status = self
                .child
                .try_wait()
                .map_err(|source| SubprocessError::Wait {
                    program: self.program.clone(),
                    source,
                })?;
            match status {
                Some(status) if status.success() => return Ok(()),
                Some(status) => {
                    return Err(SubprocessError::Failed {
                        program: self.program.clone(),
                        status,
                    });
                }
                None => {}
            }
            let now = Instant::now();
            if now >= deadline {
                self.terminate();
                return Err(SubprocessError::TimedOut {
                    program: self.program.clone(),
                });
            }
            thread::sleep(POLL_INTERVAL.min(deadline - now));
        }
    }

    pub(crate) fn terminate(&mut self) {
        #[cfg(unix)]
        let _ = Command::new("kill")
            .args(["-KILL", "--", &format!("-{}", self.child.id())])
            .output();
        #[cfg(windows)]
        let _ = Command::new("taskkill")
            .args(["/F", "/T", "/PID", &self.child.id().to_string()])
            .output();
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

impl Drop for Process {
    fn drop(&mut self) {
        if !matches!(self.child.try_wait(), Ok(Some(_))) {
            self.terminate();
        }
    }
}

pub(crate) fn run(command: &mut Command, deadline: Instant) -> Result<(), SubprocessError> {
    Process::spawn(command)?.wait_until(deadline)
}
