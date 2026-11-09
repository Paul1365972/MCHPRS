mod compile;
mod input;
mod numbers;
mod test;

use anyhow::Error;
use clap::{Parser, Subcommand};
use std::io;
use std::path::PathBuf;
use std::process::ExitCode;

const VERSION: &str = concat!(env!("CARGO_PKG_VERSION"), " (", env!("GIT_HASH"), ")");

/// Whether the command's claim holds
type Outcome = Result<bool, Error>;

#[derive(Parser)]
#[command(version = VERSION)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Run the RIL tests of a file, or of every RIL file under a directory
    Test {
        /// A RIL file or a directory
        path: PathBuf,
        /// Replace the expected circuit of each failing test with its result
        #[arg(long)]
        update: bool,
        /// Print each test's circuit after every stage
        #[arg(long)]
        print_after_all: bool,
    },
}

fn main() -> ExitCode {
    tracing_subscriber::fmt().with_writer(io::stderr).init();

    let outcome = match Cli::parse().command {
        Command::Test {
            path,
            update,
            print_after_all,
        } => test::run_tests(&path, update, print_after_all),
    };
    match outcome {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::from(1),
        Err(error) => {
            eprintln!("error: {error:#}");
            ExitCode::from(2)
        }
    }
}
