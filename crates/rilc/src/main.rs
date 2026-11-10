mod compile;
mod diff;
mod dot;
mod input;
mod numbers;
mod oracle;
mod run;
mod runner;
mod scenario;
mod setup;
mod test;
mod update;

use anyhow::Error;
use clap::error::ErrorKind;
use clap::{CommandFactory, Parser, Subcommand};
use setup::Setup;
use std::io;
use std::num::NonZeroUsize;
use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Duration;

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
    /// Compile a world or a RIL circuit and write the result as RIL, or as Graphviz DOT
    #[command(after_help = setup::help())]
    Compile {
        /// A schematic, a plot file or a RIL file
        input: PathBuf,
        /// The setup to compile for, like "direct -o"; defaults to the first backend
        #[arg(short = 'b', long = "backend", value_name = "SETUP", value_parser = Setup::parse)]
        setup: Option<Setup>,
        /// The circuit to compile, if the RIL file holds several
        #[arg(long, value_name = "NAME")]
        circuit: Option<String>,
        /// Write the circuit after every stage, each named after its stage
        #[arg(long)]
        print_after_all: bool,
        /// Write Graphviz DOT instead of RIL
        #[arg(long)]
        dot: bool,
        /// Write to FILE instead of stdout
        #[arg(short, long, value_name = "FILE")]
        output: Option<PathBuf>,
    },
    /// Play a scenario of a world or a RIL circuit on a setup running by itself and time it
    #[command(after_help = setup::help())]
    Run {
        /// The scenario, a TOML file
        scenario: PathBuf,
        /// The setup to run on, like "direct -o"
        #[arg(short = 'b', long = "backend", value_name = "SETUP", value_parser = Setup::parse)]
        setup: Setup,
        /// Play the scenario this many times, each on a fresh engine
        #[arg(long, value_name = "N", default_value = "1")]
        repeat: NonZeroUsize,
        /// After the scenario, run without a tick limit for SECONDS and print that rate too
        #[arg(long, value_name = "SECONDS", value_parser = run::seconds)]
        unlimited: Option<Duration>,
        /// Write the final state of every output port to FILE
        #[arg(long, value_name = "FILE")]
        save_outputs: Option<PathBuf>,
    },
    /// Play a scenario on two setups and show the first tick after which a component that both
    /// show differs
    #[command(after_help = setup::help())]
    Diff {
        /// The scenario, a TOML file
        scenario: PathBuf,
        /// Each of the two setups, like -b direct -b "direct -o"
        #[arg(
            short = 'b',
            long = "backend",
            value_name = "SETUP",
            value_parser = Setup::parse,
            required = true
        )]
        setups: Vec<Setup>,
    },
    /// Update every block of a schematic or plot, run it until no tick is pending and write it as
    /// a schematic
    Update {
        /// A schematic or a plot file
        input: PathBuf,
        /// The schematic to write
        output: PathBuf,
        /// Fail if ticks are still pending after N ticks
        #[arg(long, value_name = "N", default_value = "10000")]
        settle_limit: u64,
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
        Command::Compile {
            input,
            setup,
            circuit,
            print_after_all,
            dot,
            output,
        } => compile::command(
            &input,
            &setup.unwrap_or_default(),
            circuit.as_deref(),
            print_after_all,
            dot,
            output.as_deref(),
        ),
        Command::Run {
            scenario,
            setup,
            repeat,
            unlimited,
            save_outputs,
        } => run::command(
            &scenario,
            &setup,
            repeat,
            unlimited,
            save_outputs.as_deref(),
        ),
        Command::Diff { scenario, setups } => {
            let [first, second] = two_setups(setups);
            diff::command(&scenario, [&first, &second])
        }

        Command::Update {
            input,
            output,
            settle_limit,
        } => update::command(&input, &output, settle_limit),
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

fn two_setups(setups: Vec<Setup>) -> [Setup; 2] {
    let Ok(setups) = <[Setup; 2]>::try_from(setups) else {
        let mut cli = Cli::command();
        cli.build();
        cli.find_subcommand_mut("diff")
            .expect("diff is a subcommand")
            .error(
                ErrorKind::WrongNumberOfValues,
                "diff takes exactly two setups",
            )
            .exit();
    };
    setups
}
