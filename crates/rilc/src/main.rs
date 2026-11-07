use clap::{Parser, Subcommand};
use compile::Compilation;
use mchprs_blocks::BlockPos;
use mchprs_redpiler::compile::{optimize, BuildContext};
use mchprs_redpiler::extract::{extract, ExtractOptions};
use mchprs_redpiler::graph::Graph;
use mchprs_redpiler::passes::{self, Pass};
use mchprs_redpiler_backends::{BackendSetup, BACKENDS};
use mchprs_ril::{Module, Printed};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

mod compile;
mod test;

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
        /// A comma separated list of passes to run instead of the backend's
        #[arg(long, value_delimiter = ',')]
        passes: Option<Vec<String>>,
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

fn extract_schematic(
    ril_file_path: &Path,
    schematic_path: &str,
    options: &ExtractOptions,
) -> Option<Graph> {
    let schematic_path = ril_file_path.parent().unwrap().join(schematic_path);
    let Some(schematic) = fs::read(&schematic_path)
        .ok()
        .and_then(|bytes| mchprs_schematic::read(&bytes).ok())
    else {
        eprintln!(
            "error: failed to load schematic at path: {:?}",
            schematic_path
        );
        return None;
    };
    for (name, count) in &schematic.unknown_blocks {
        eprintln!(
            "warning: replaced {count} block(s) of unknown or invalid state `{name}` with air"
        );
    }
    let origin = schematic.volume.region().min();
    let volume = schematic.volume.translated(BlockPos::zero() - origin);
    match extract(&volume, volume.region(), &[], options) {
        Ok(graph) => Some(graph),
        Err(error) => {
            eprintln!("error: {error}");
            None
        }
    }
}

fn run_passes(passes: &[&dyn Pass], mut graph: Graph, print_after_all: bool) -> Graph {
    for &pass in passes {
        optimize(&mut graph, &[pass], &mut BuildContext::silent())
            .expect("a silent build is never cancelled");
        if print_after_all {
            println!("{}", Printed::new(&graph, &pass.name().replace('-', "_")));
        }
    }
    graph
}

fn load_ril(path: &Path, src: &str) -> Option<Module> {
    match Module::parse(src) {
        Ok(module) => Some(module),
        Err(error) => {
            eprintln!("error: {}:{error}", path.display());
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
) -> Result<(), String> {
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
    Ok(())
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

    match cli.command {
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
        } => {
            if let Err(error) = compile_command(
                &input_path,
                &output_path,
                backend.unwrap_or_default(),
                print_after_all,
                passes,
            ) {
                eprintln!("error: {error}");
                return ExitCode::FAILURE;
            }
        }
        Command::Version => {
            println!("{}", get_version_string());
        }
    }
    ExitCode::SUCCESS
}
