use std::env;
use std::fs::{File, TryLockError};
use std::io;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::thread;
use std::time::{Duration, Instant};

const ROOT_VARIABLE: &str = "MCHPRS_FPGA_CACHE";
const POLL_INTERVAL: Duration = Duration::from_millis(25);

static BUILD_SEQUENCE: AtomicU64 = AtomicU64::new(0);

fn configured_root() -> Option<PathBuf> {
    env::var_os(ROOT_VARIABLE).map(PathBuf::from)
}

pub(crate) fn board_root() -> PathBuf {
    configured_root().unwrap_or_else(|| env::temp_dir().join("mchprs-fpga"))
}

pub(crate) fn verilator_root() -> PathBuf {
    configured_root().map_or_else(
        || env::temp_dir().join("mchprs-verilator"),
        |root| root.join("verilator"),
    )
}

pub(crate) fn unique_build_directory(root: &Path, prefix: &str, identity: u64) -> PathBuf {
    root.join(format!(
        "{prefix}{identity:016x}-{}-{}",
        std::process::id(),
        BUILD_SEQUENCE.fetch_add(1, Ordering::Relaxed)
    ))
}

pub(crate) fn lock(file: &File, deadline: Instant) -> io::Result<()> {
    loop {
        match file.try_lock() {
            Ok(()) => return Ok(()),
            Err(TryLockError::WouldBlock) => {}
            Err(TryLockError::Error(error)) => return Err(error),
        }
        let now = Instant::now();
        if now >= deadline {
            return Err(io::Error::new(io::ErrorKind::TimedOut, "timed out"));
        }
        thread::sleep(POLL_INTERVAL.min(deadline - now));
    }
}
