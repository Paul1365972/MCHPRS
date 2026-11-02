mod compile;
mod experiment;
mod test;

use clap::{Args, Parser, Subcommand};
use experiment::EngineKind;
use line_index::{LineIndex, TextSize};
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::BlockPos;
use mchprs_redpiler::{
    compile::CompileOptions,
    extract::extract,
    graph::Graph,
    passes::{self, Pass},
};
use mchprs_ril::{DumpGraph, RILModule, RilOptions};
use std::fs;
use std::num::NonZeroU64;
use std::ops::ControlFlow;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

#[derive(Parser)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Args)]
struct CompileFlags {
    /// Enable optimization passes which may significantly increase compile times.
    #[arg(long, short = 'O')]
    optimize: bool,
    /// Let optimizations reorder updates within a tick
    #[arg(long, short)]
    reorder: bool,
    /// Only flush lamp, button, lever, pressure plate, or trapdoor updates.
    #[arg(long, short)]
    io_only: bool,
    /// Consider a redstone dot to be an output block (for color screens)
    #[arg(long, short = 'd')]
    wire_dot_out: bool,
    /// Consider "illegal" redstone wires to be an output block (for sprite screens)
    #[arg(long, short = 'l')]
    illegal_states_out: bool,
    /// Consider a redstone cross to be an output block (for sprite screens without illegal wire states)
    #[arg(long, short = 'c')]
    wire_cross_out: bool,
}

impl CompileFlags {
    fn ril_options(&self) -> RilOptions {
        RilOptions {
            optimize: self.optimize,
            reorder: self.reorder,
            io_only: self.io_only,
            wire_dot_out: self.wire_dot_out,
            illegal_states_out: self.illegal_states_out,
            wire_cross_out: self.wire_cross_out,
            ..Default::default()
        }
    }
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
    Compile {
        /// Path to the input file
        input_path: PathBuf,

        /// Path to the output file
        #[arg(short = 'o')]
        output_path: Option<PathBuf>,

        #[command(flatten)]
        flags: CompileFlags,
        /// Print out the RIL circuit after every redpiler pass
        #[arg(long)]
        print_after_all: bool,
        /// A comma separated list of passes to run. This can only be used by the rilc driver.
        #[arg(long)]
        passes: Option<String>,
    },
    /// Run a circuit file or schematic on an engine and measure it
    Run {
        /// Path to the circuit file or schematic
        input_path: PathBuf,

        #[arg(long, default_value_t = EngineKind::Direct)]
        engine: EngineKind,
        #[arg(long)]
        ticks: u64,
        #[command(flatten)]
        flags: CompileFlags,
    },
    /// Run a circuit file or schematic on two engines and explain the first difference
    Diff {
        /// Path to the circuit file or schematic
        input_path: PathBuf,

        #[arg(long, num_args = 2, action = clap::ArgAction::Set, default_values_t = [EngineKind::Direct, EngineKind::Direct])]
        engines: Vec<EngineKind>,
        #[arg(long)]
        ticks: u64,
        /// Drive the inputs one at a time, one every DRIVE_INTERVAL ticks
        #[arg(long)]
        drive_interval: Option<NonZeroU64>,
        #[command(flatten)]
        flags: CompileFlags,
    },
    Version,
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

fn extract_schematic(schematic_path: &Path, options: &CompileOptions) -> Option<Graph> {
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

/// The passes `options` name, or else the standard passes of its compile options.
fn chosen_passes(options: &RilOptions) -> Option<Vec<&'static dyn Pass>> {
    let Some(names) = &options.passes else {
        return Some(passes::standard(&options.compile_options()));
    };
    names
        .split(',')
        .map(|name| {
            let pass = passes::named(name);
            if pass.is_none() {
                eprintln!("error: there is no pass `{name}`");
            }
            pass
        })
        .collect()
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
            flags,
            print_after_all,
            passes,
        } => {
            let options = RilOptions {
                print_after_all,
                passes,
                ..flags.ril_options()
            };
            compile::compile(&input_path, &output_path, &options);
            Ok(true)
        }
        Command::Run {
            input_path,
            engine,
            ticks,
            flags,
        } => experiment::load_circuit(&input_path, &flags.ril_options())
            .and_then(|circuit| experiment::run(circuit, engine, ticks))
            .map(|()| true),
        Command::Diff {
            input_path,
            engines,
            ticks,
            drive_interval,
            flags,
        } => {
            let engines = <[EngineKind; 2]>::try_from(engines).expect("clap takes two engines");
            experiment::load_circuit(&input_path, &flags.ril_options())
                .and_then(|circuit| experiment::diff(circuit, engines, ticks, drive_interval))
        }
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
