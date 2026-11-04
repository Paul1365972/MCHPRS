mod compile;
mod experiment;
mod test;

use clap::{Parser, Subcommand};
use compile::Compilation;
use experiment::{Configured, Source};
use line_index::{LineIndex, TextSize};
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::BlockPos;
use mchprs_redpiler::extract::{extract, ExtractOptions};
use mchprs_redpiler::graph::Graph;
use mchprs_redpiler::passes::{self, Pass};
use mchprs_redpiler_backends::{BackendSetup, BACKENDS};
use mchprs_ril::{DumpGraph, RILModule};
use std::error::Error;
use std::fs;
use std::num::NonZeroU64;
use std::ops::ControlFlow;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

type Outcome = Result<bool, Box<dyn Error + Send + Sync>>;

#[derive(Parser)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    Test {
        /// Path to the test or test directory
        path: PathBuf,

        #[arg(long)]
        update: bool,
    },
    /// Compile a RIL file to RIL, or a schematic to a circuit file
    #[command(after_help = backends_help())]
    Compile {
        /// Path to the input file
        input_path: PathBuf,

        /// Path to the output file
        #[arg(long = "output")]
        output_path: Option<PathBuf>,

        /// Backend and flags to compile for, like "direct -o"; defaults to the first backend
        #[arg(short, long, value_parser = BackendSetup::parse)]
        backend: Option<BackendSetup>,
        /// Print out the RIL circuit after every redpiler pass
        #[arg(long)]
        print_after_all: bool,
        /// Comma-separated passes to run instead of the backend's
        #[arg(long, value_delimiter = ',')]
        passes: Option<Vec<String>>,
    },
    /// Run a circuit file or schematic on a backend and measure it
    #[command(after_help = backends_help())]
    Run {
        /// Path to the circuit file or schematic
        input_path: PathBuf,

        /// Backend and flags to run on, like "direct -o"; defaults to the first backend
        #[arg(short, long, value_parser = BackendSetup::parse)]
        backend: Option<BackendSetup>,
        #[arg(long)]
        ticks: u64,
    },
    /// Run a circuit file or schematic on two backends and explain the first difference
    #[command(after_help = backends_help())]
    Diff {
        /// Path to the circuit file or schematic
        input_path: PathBuf,

        /// The two backends and their flags, like "direct" "direct -o"
        #[arg(long, num_args = 2, action = clap::ArgAction::Set, required = true, value_parser = BackendSetup::parse)]
        backends: Vec<BackendSetup>,
        #[arg(long)]
        ticks: u64,
        /// Drive the inputs one at a time, one every DRIVE_INTERVAL ticks
        #[arg(long)]
        drive_interval: Option<NonZeroU64>,
    },
    Version,
}

fn backends_help() -> String {
    let flags = || BACKENDS.iter().flat_map(|backend| backend.flags());
    let width = flags()
        .map(|flag| flag.to_string().len())
        .max()
        .unwrap_or(0);
    let mut help = "Backends and their flags:".to_owned();
    for backend in BACKENDS {
        help.push_str(&format!("\n  {}", backend.name()));
        for flag in backend.flags() {
            let usage = flag.to_string();
            help.push_str(&format!("\n    {usage:width$}  {}", flag.description()));
        }
    }
    help
}

fn read_schematic_at_origin(bytes: &[u8]) -> Result<BlockVolume, String> {
    let schematic = mchprs_schematic::read::read(bytes).map_err(|error| error.to_string())?;
    for (name, count) in &schematic.unknown_blocks {
        eprintln!(
            "warning: replaced {count} block(s) of unknown or invalid state `{name}` with air"
        );
    }
    let origin = schematic.volume.region().min();
    Ok(schematic.volume.translated(BlockPos::zero() - origin))
}

fn extract_schematic(schematic_path: &Path, options: &ExtractOptions) -> Option<Graph> {
    let volume = match fs::read(schematic_path)
        .map_err(|error| error.to_string())
        .and_then(|bytes| read_schematic_at_origin(&bytes))
    {
        Ok(volume) => volume,
        Err(error) => {
            eprintln!(
                "error: failed to load the schematic at {}: {error}",
                schematic_path.display()
            );
            return None;
        }
    };
    match extract(&volume, volume.region(), &[], options) {
        Ok(graph) => Some(graph),
        Err(error) => {
            eprintln!("error: {error}");
            None
        }
    }
}

fn run_passes(passes: &[&dyn Pass], mut graph: Graph, print_after_all: bool) -> Graph {
    let _ = passes::run_passes(passes, &mut graph, &mut |pass, graph| {
        if print_after_all {
            println!("Circuit after pass: {}", pass.name());
            graph.dump();
        }
        ControlFlow::Continue(())
    });
    graph
}

fn load_ril(path: &Path, src: &str) -> Option<RILModule> {
    match RILModule::parse_from_string(src) {
        Ok(module) => Some(module),
        Err(err) => {
            eprintln!("error: failed to load RIL module at path: {:?}", path);
            let file_name = path.file_name().unwrap();
            let line_index = LineIndex::new(src);
            let pos = TextSize::new(err.pos as u32);
            let line_col = line_index.line_col(pos);
            eprintln!(
                "{}:{}:{} {}",
                file_name.display(),
                line_col.line + 1,
                line_col.col + 1,
                err.message
            );
            None
        }
    }
}

fn named_passes(names: &[String]) -> Result<Vec<&'static dyn Pass>, String> {
    names
        .iter()
        .map(|name| passes::named(name).ok_or_else(|| format!("unknown pass `{name}`")))
        .collect()
}

fn compile_command(
    input_path: &Path,
    output_path: &Option<PathBuf>,
    backend: BackendSetup,
    print_after_all: bool,
    passes: Option<Vec<String>>,
) -> Outcome {
    let plan = backend.configure()?;
    let passes = match passes {
        Some(names) => named_passes(&names)?,
        None => plan.passes,
    };
    let compilation = Compilation {
        extract: &plan.extract,
        passes: &passes,
        print_after_all,
    };
    compile::compile(input_path, output_path, &compilation);
    Ok(true)
}

fn run_command(input_path: &Path, backend: BackendSetup, ticks: u64) -> Outcome {
    let backend = Configured::new(backend)?;
    let source = Source::read(input_path)?;
    experiment::run(&source, backend, ticks)?;
    Ok(true)
}

fn diff_command(
    input_path: &Path,
    backends: Vec<BackendSetup>,
    ticks: u64,
    drive_interval: Option<NonZeroU64>,
) -> Outcome {
    let backends = backends
        .into_iter()
        .map(Configured::new)
        .collect::<Result<Vec<_>, _>>()?;
    let Ok(backends) = <[Configured; 2]>::try_from(backends) else {
        panic!("clap takes two backends");
    };
    let source = Source::read(input_path)?;
    experiment::diff(&source, backends, ticks, drive_interval)
}

pub fn get_version_string() -> String {
    format!(
        "{} v{}\n{}",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION"),
        env!("GIT_HASH")
    )
}

fn main() -> ExitCode {
    tracing_subscriber::fmt::init();

    let cli = Cli::parse();

    let result = match cli.command {
        Command::Test { path, update } => {
            return if test::run_tests(path, update) {
                ExitCode::SUCCESS
            } else {
                ExitCode::FAILURE
            };
        }
        Command::Compile {
            input_path,
            output_path,
            backend,
            print_after_all,
            passes,
        } => compile_command(
            &input_path,
            &output_path,
            backend.unwrap_or_default(),
            print_after_all,
            passes,
        ),
        Command::Run {
            input_path,
            backend,
            ticks,
        } => run_command(&input_path, backend.unwrap_or_default(), ticks),
        Command::Diff {
            input_path,
            backends,
            ticks,
            drive_interval,
        } => diff_command(&input_path, backends, ticks, drive_interval),
        Command::Version => {
            println!("{}", get_version_string());
            Ok(true)
        }
    };
    match result {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(error) => {
            eprintln!("error: {error}");
            ExitCode::FAILURE
        }
    }
}
