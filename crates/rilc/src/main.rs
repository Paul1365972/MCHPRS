use clap::{Parser, Subcommand};
use line_index::{LineIndex, TextSize};
use mchprs_blocks::BlockPos;
use mchprs_redpiler::{
    passes::{PassPipeline, PassPipelineBuilder, PassRegistry},
    ril::RILModule,
    CompilerOptions, Reporting,
};
use mchprs_schematic::{load_schematic, paste_clipboard};
use mchprs_world::testing::TestWorld;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

mod compile;
mod diff;
mod experiment;
mod run;
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
        /// While running, only report changes of input and output blocks.
        #[arg(long, short)]
        io_only: bool,
        /// Also limit snapshots, including the one when stopping, to input and output blocks.
        #[arg(long)]
        io_snapshots: bool,
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
    /// Run an experiment file on one backend and print a JSON report
    Run {
        experiment: PathBuf,
        #[arg(long, default_value = "direct")]
        backend: String,
        /// Write the report to this file instead of stdout
        #[arg(short = 'o')]
        output: Option<PathBuf>,
    },
    /// List the compiled inputs and outputs of an experiment's plot
    Inputs {
        experiment: PathBuf,
    },
    /// Write the Verilog that the board backend would synthesize for an experiment's plot
    #[cfg(feature = "fpga")]
    Verilog {
        experiment: PathBuf,
        /// Directory to write circuit.v and the RTL library into
        #[arg(short = 'o')]
        output: PathBuf,
    },
    /// Run an experiment on several backends side by side and report the first difference
    Diff {
        experiment: PathBuf,
        /// Comma separated backends, for example direct,phase
        #[arg(long, value_delimiter = ',', default_value = "direct,phase")]
        backends: Vec<String>,
        /// Compare every this many ticks
        #[arg(long, default_value_t = 1)]
        every: u64,
        /// Compare only what the world shows, not pending ticks
        #[arg(long)]
        ignore_pending: bool,
        /// Write the report to this file instead of stdout
        #[arg(short = 'o')]
        output: Option<PathBuf>,
    },
    Version,
}

fn report_output(path: Option<&Path>) -> std::io::Result<Box<dyn std::io::Write>> {
    Ok(match path {
        Some(path) => Box::new(std::io::BufWriter::new(std::fs::File::create(path)?)),
        None => Box::new(std::io::stdout().lock()),
    })
}

fn load_world(
    ril_file_path: &Path,
    schem_path: &Option<String>,
) -> Option<(TestWorld, (BlockPos, BlockPos))> {
    Some(if let Some(schem_path) = schem_path {
        let schem_path = ril_file_path.parent().unwrap().join(schem_path);
        let Ok(schematic) = load_schematic(&schem_path) else {
            eprintln!("error: failed to load schematic at path: {:?}", schem_path);
            return None;
        };
        let x_size = schematic.size_x.div_ceil(16) as i32;
        let y_size = schematic.size_y.div_ceil(16) as i32;
        let z_size = schematic.size_z.div_ceil(16) as i32;
        let mut world = TestWorld::new(x_size, y_size, z_size);
        paste_clipboard(&mut world, &schematic, BlockPos::zero(), true);

        let bounds = (
            BlockPos::zero(),
            BlockPos::new(
                schematic.size_x as i32 - 1,
                schematic.size_y as i32 - 1,
                schematic.size_z as i32 - 1,
            ),
        );

        (world, bounds)
    } else {
        (
            TestWorld::new(1, 1, 1),
            (BlockPos::zero(), BlockPos::zero()),
        )
    })
}

fn load_ril(path: &Path, src: &str) -> Option<RILModule> {
    match RILModule::parse_from_string(src) {
        Ok(module) => Some(module),
        Err(err) => {
            eprintln!("error: failed to load RIL module at path: {:?}", &path);
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

fn parse_pass_pipeline<'p>(
    registry: &'p PassRegistry<TestWorld>,
    passes: &str,
) -> Option<PassPipeline<'p, TestWorld>> {
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
    tracing_subscriber::fmt()
        .with_writer(std::io::stderr)
        .init();

    match execute(Cli::parse().command) {
        Ok(code) => code,
        Err(error) => {
            eprintln!("error: {}", mchprs_utils::ErrorChain(&*error));
            ExitCode::FAILURE
        }
    }
}

fn execute(command: Command) -> experiment::Result<ExitCode> {
    match command {
        Command::Test { path, update } => {
            return Ok(if test::run_tests(path, update) {
                ExitCode::SUCCESS
            } else {
                ExitCode::FAILURE
            });
        }
        Command::Compile {
            input_path,
            output_path,
            optimize,
            reorder,
            export,
            io_only,
            io_snapshots,
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
                reporting: if io_snapshots {
                    Reporting::InputsAndOutputs
                } else if io_only {
                    Reporting::InputsAndOutputsWhileRunning
                } else {
                    Reporting::All
                },
                update: false,
                wire_dot_out,
                illegal_states_out,
                wire_cross_out,
                print_after_all,
                print_before_backend: false,
                passes,
                backend: None,
            };
            compile::compile(&input_path, &output_path, &options);
        }
        Command::Run {
            experiment,
            backend,
            output,
        } => {
            let mut output = report_output(output.as_deref())?;
            run::run(&experiment, &backend, output.as_mut())?;
        }
        Command::Inputs { experiment } => experiment::list_inputs(&experiment)?,
        #[cfg(feature = "fpga")]
        Command::Verilog { experiment, output } => {
            experiment::write_verilog(&experiment, &output)?;
        }
        Command::Diff {
            experiment,
            backends,
            every,
            ignore_pending,
            output,
        } => {
            let mut output = report_output(output.as_deref())?;
            if !diff::diff(
                &experiment,
                &backends,
                every,
                ignore_pending,
                output.as_mut(),
            )? {
                return Ok(ExitCode::FAILURE);
            }
        }
        Command::Version => {
            println!("{}", get_version_string());
        }
    }
    Ok(ExitCode::SUCCESS)
}
