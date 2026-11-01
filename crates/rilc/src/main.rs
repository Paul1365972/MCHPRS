use clap::{Parser, Subcommand};
use line_index::{LineIndex, TextSize};
use mchprs_blocks::BlockPos;
use mchprs_redpiler::{
    compile_graph::CompileGraph,
    extract::extract,
    passes::{PassPipeline, PassPipelineBuilder, PassRegistry},
    ril::RILModule,
    CompilerOptions,
};
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
    Compile {
        /// Path to the input file
        input_path: PathBuf,

        /// Path to the output file
        #[arg(short = 'o')]
        output_path: Option<PathBuf>,

        /// Enable optimization passes which may significantly increase compile times.
        #[arg(long, short = 'O')]
        optimize: bool,
        /// Let optimizations reorder updates within a tick
        #[arg(long, short)]
        reorder: bool,
        /// Export the graph to a binary format. See the [`redpiler_graph`] crate.
        #[arg(long, short)]
        export: bool,
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
        /// Print out the RIL circuit after every redpiler pass
        #[arg(long)]
        print_after_all: bool,
        /// A comma separated list of passes to run. This can only be used by the rilc driver.
        #[arg(long)]
        passes: Option<String>,
    },
    Version,
}

fn extract_schematic(
    ril_file_path: &Path,
    schematic_path: &str,
    options: &CompilerOptions,
) -> Option<CompileGraph> {
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
    Some(extract(&volume, volume.region(), &[], options))
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

fn parse_pass_pipeline<'p>(registry: &'p PassRegistry, passes: &str) -> Option<PassPipeline<'p>> {
    let mut builder = PassPipelineBuilder::new(registry);
    for driver_key in passes.split(',') {
        if !builder.add_pass_by_driver_key(driver_key) {
            eprintln!("error: failed to add pass with key: {}", driver_key);
            return None;
        }
    }
    Some(builder.build())
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
            optimize,
            reorder,
            export,
            io_only,
            wire_dot_out,
            illegal_states_out,
            wire_cross_out,
            print_after_all,
            passes,
        } => {
            let options = CompilerOptions {
                optimize,
                reorder,
                export,
                io_only,
                update: false,
                export_dot_graph: false,
                wire_dot_out,
                illegal_states_out,
                wire_cross_out,
                print_after_all,
                print_before_backend: false,
                backend_variant: Default::default(),
                passes,
            };
            compile::compile(&input_path, &output_path, &options);
        }
        Command::Version => {
            println!("{}", get_version_string());
        }
    }
    ExitCode::SUCCESS
}
