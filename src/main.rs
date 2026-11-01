use mchprs_core::server::{get_version_string, MinecraftServer};
use std::fs::{self, File};
use std::io;
use std::path::Path;
use std::process::ExitCode;
use std::sync::Mutex;
use tracing::warn;
use tracing_subscriber::filter::{LevelFilter, Targets};
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt;
use tracing_subscriber::{fmt, Layer};

const LOGS: &str = "./logs";

fn main() -> ExitCode {
    match std::env::args().nth(1).as_deref() {
        None => {}
        Some("-v" | "--version") => {
            println!("{}", get_version_string());
            return ExitCode::SUCCESS;
        }
        Some(argument) => {
            eprintln!("error: unknown argument `{argument}`");
            eprintln!("usage: mchprs [--version]");
            return ExitCode::FAILURE;
        }
    }

    let log_file = match open_log_file() {
        Ok(file) => file,
        Err(error) => {
            eprintln!("Failed to open {LOGS}/latest.log: {error}");
            return ExitCode::FAILURE;
        }
    };
    let (filter, filter_error) = log_filter();
    tracing_subscriber::registry()
        .with(
            fmt::layer()
                .log_internal_errors(true)
                .with_filter(filter.clone()),
        )
        .with(
            fmt::layer()
                .log_internal_errors(true)
                .with_ansi(false)
                .with_writer(Mutex::new(log_file))
                .with_filter(filter),
        )
        .init();
    if let Some(error) = filter_error {
        warn!("Invalid MCHPRS_LOG, using `info`: {error}");
    }

    MinecraftServer::run();
    ExitCode::SUCCESS
}

fn log_filter() -> (Targets, Option<String>) {
    let default = Targets::new().with_default(LevelFilter::INFO);
    let value = std::env::var("MCHPRS_LOG").unwrap_or_default();
    let directives: Vec<&str> = value
        .split(',')
        .map(str::trim)
        .filter(|directive| !directive.is_empty())
        .collect();
    if directives.is_empty() {
        return (default, None);
    }
    match directives.join(",").parse::<Targets>() {
        Ok(targets) => (targets, None),
        Err(error) => (default, Some(error.to_string())),
    }
}

fn open_log_file() -> io::Result<File> {
    fs::create_dir_all(LOGS)?;
    let latest = Path::new(LOGS).join("latest.log");
    if latest.exists() {
        let mut number = 1u32;
        while Path::new(LOGS).join(format!("{number}.log")).exists() {
            number += 1;
        }
        fs::rename(&latest, Path::new(LOGS).join(format!("{number}.log")))?;
    }
    File::create(latest)
}
