mod compile;
mod diff;
mod fuzz;
mod input;
mod numbers;
mod run;
mod scenario;
mod setup;
mod subject;
mod test;

use clap::error::ErrorKind;
use clap::{CommandFactory, Parser, Subcommand};
use setup::Setup;
use std::num::{NonZeroU64, NonZeroUsize};
use std::path::PathBuf;
use std::process::ExitCode;

type Error = Box<dyn std::error::Error + Send + Sync>;

/// Whether the command's claim holds, or why the command couldn't run
type Outcome = Result<bool, Error>;

const VERSION: &str = concat!(env!("CARGO_PKG_VERSION"), " (", env!("GIT_HASH"), ")");

/// Compiles, runs, compares and tests Redpiler circuits.
///
/// Exits with 0 when the command's claim holds, 1 when it doesn't (a test fails, two setups
/// differ), and 2 when it can't run.
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
    /// Compile a world or a RIL circuit and write the result as RIL
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
        /// Write the RIL to FILE instead of stdout
        #[arg(short, long, value_name = "FILE")]
        output: Option<PathBuf>,
    },
    /// Play a scenario on a setup and time it
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
        /// Write the final state of every output port to FILE
        #[arg(long, value_name = "FILE")]
        save_outputs: Option<PathBuf>,
    },
    /// Play a scenario on two setups and explain where they first differ
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
    /// Play random scenarios on random circuits, or on an input, under two setups and name the
    /// cases on which they differ by seed
    #[command(after_help = setup::help())]
    Fuzz {
        /// A schematic, a plot file or a RIL file of one circuit to play random scenarios on,
        /// instead of random circuits
        input: Option<PathBuf>,
        /// Each of the two setups, like -b direct -b "direct -o"
        #[arg(
            short = 'b',
            long = "backend",
            value_name = "SETUP",
            value_parser = Setup::parse,
            required = true
        )]
        setups: Vec<Setup>,
        /// The first case's seed; each next case takes the next seed
        #[arg(long, value_name = "S", default_value = "1")]
        seed: u64,
        /// How many cases to play
        #[arg(long, value_name = "N", default_value = "1000")]
        count: NonZeroU64,
        /// How many ticks each case runs
        #[arg(long, value_name = "N", default_value = "300")]
        ticks: u64,
        /// Write each differing case to DIR as <seed>.toml, a scenario for diff, and a random
        /// circuit as <seed>.ril
        #[arg(long, value_name = "DIR")]
        out: Option<PathBuf>,
    },
}

fn two_setups(command: &str, setups: Vec<Setup>) -> [Setup; 2] {
    let Ok(setups) = <[Setup; 2]>::try_from(setups) else {
        let mut cli = Cli::command();
        cli.build();
        cli.find_subcommand_mut(command)
            .expect("the command is a subcommand")
            .error(
                ErrorKind::WrongNumberOfValues,
                format!("{command} takes exactly two setups"),
            )
            .exit();
    };
    setups
}

fn main() -> ExitCode {
    tracing_subscriber::fmt()
        .with_writer(std::io::stderr)
        .init();

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
            output,
        } => compile::command(
            &input,
            &setup.unwrap_or_default(),
            circuit.as_deref(),
            print_after_all,
            output.as_deref(),
        ),
        Command::Run {
            scenario,
            setup,
            repeat,
            save_outputs,
        } => run::command(&scenario, &setup, repeat, save_outputs.as_deref()),
        Command::Diff { scenario, setups } => {
            let [first, second] = two_setups("diff", setups);
            diff::command(&scenario, [&first, &second])
        }
        Command::Fuzz {
            input,
            setups,
            seed,
            count,
            ticks,
            out,
        } => {
            let [first, second] = two_setups("fuzz", setups);
            let cases = fuzz::Cases { seed, count, ticks };
            fuzz::command(input.as_deref(), [&first, &second], &cases, out.as_deref())
        }
    };
    match outcome {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::from(1),
        Err(error) => {
            eprintln!("error: {error}");
            ExitCode::from(2)
        }
    }
}
