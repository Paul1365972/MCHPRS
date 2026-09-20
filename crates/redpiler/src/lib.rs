pub mod backend;
mod block_map;
pub mod compile_graph;
mod engine;
mod netlist;
pub mod passes;
mod progress;
pub mod ril;
pub mod string_replacer;
mod worker;

pub use backend::{Backend, BackendFailure, Batch, Snapshot};
pub use block_map::BlockMap;
pub use engine::{Engine, Input, NodeChanges, PendingTick};
pub use netlist::{Link, Netlist, NodeId};
pub use progress::{CompileProgress, PassStatus};
pub use worker::Worker;

use crate::compile_graph::CompileGraph;
use crate::passes::PassRegistry;
use mchprs_blocks::BlockPos;
use mchprs_world::World;
use std::time::Instant;
use tracing::{debug, info, warn};

#[derive(Default, PartialEq, Eq, Debug, Clone)]
pub struct CompilerOptions {
    /// Enable optimization passes which may significantly increase compile times.
    pub optimize: bool,
    /// Export the graph to a binary format. See the [`redpiler_graph`] crate.
    pub export: bool,
    /// While running, only report changes of input and output blocks. Stop reports every block.
    pub io_only: bool,
    /// Update all blocks in the input region after reset.
    pub update: bool,
    /// Consider a redstone dot to be an output block (for color screens)
    pub wire_dot_out: bool,
    /// Consider "illegal" redstone wires to be an output block (for sprite screens)
    pub illegal_states_out: bool,
    /// Consider a redstone cross to be an output block (for sprite screens without illegal wire states)
    pub wire_cross_out: bool,
    /// Print out the RIL circuit after every redpiler pass
    pub print_after_all: bool,
    /// Print out the RIL circuit before starting backend compile
    pub print_before_backend: bool,
    /// A comma separated list of passes to run. This can only be used by the rilc driver.
    pub passes: Option<String>,
}

impl CompilerOptions {
    fn parse_option(&mut self, option: &str) {
        if option.starts_with("--") {
            if let Some(passes_str) = option.strip_prefix("--passes=") {
                self.passes = Some(passes_str.to_owned());
                return;
            }

            match option {
                "--optimize" => self.optimize = true,
                "--export" => self.export = true,
                "--io-only" => self.io_only = true,
                "--update" => self.update = true,
                "--wire-dot-out" => self.wire_dot_out = true,
                "--illegal-states-out" => self.illegal_states_out = true,
                "--wire-cross-out" => self.wire_cross_out = true,
                "--print-after-all" => self.print_after_all = true,
                "--print-before-backend" => self.print_before_backend = true,
                // FIXME: use actual error handling
                _ => warn!("Unrecognized option: {}", option),
            }
        } else if let Some(str) = option.strip_prefix('-') {
            for c in str.chars() {
                let lower = c.to_lowercase().to_string();
                match lower.as_str() {
                    "o" => self.optimize = true,
                    "e" => self.export = true,
                    "i" => self.io_only = true,
                    "u" => self.update = true,
                    "d" => self.wire_dot_out = true,
                    "l" => self.illegal_states_out = true,
                    "c" => self.wire_cross_out = true,
                    // FIXME: use actual error handling
                    _ => warn!("Unrecognized option: -{}", c),
                }
            }
        } else {
            // FIXME: use actual error handling
            warn!("Unrecognized option: {}", option);
        }
    }

    pub fn parse(str: &str) -> CompilerOptions {
        let mut co: CompilerOptions = Default::default();
        let options = str.split_whitespace();
        for option in options {
            co.parse_option(option);
        }
        co
    }
}

pub struct CompilerInput<'w, W: World> {
    pub world: &'w W,
    pub bounds: (BlockPos, BlockPos),
}

pub fn compile<W: World>(
    world: &W,
    bounds: (BlockPos, BlockPos),
    options: &CompilerOptions,
    progress: &CompileProgress,
) -> Netlist {
    debug!("Starting compile");
    let start = Instant::now();

    let input = CompilerInput { world, bounds };
    let registry = PassRegistry::default();
    let pass_pipeline = passes::build_pass_pipeline::<W>(&registry, options);
    let graph = pass_pipeline.run_passes(options, &input, CompileGraph::default(), progress);

    let netlist = Netlist::from(graph);
    info!("Compile completed in {:?}", start.elapsed());
    netlist
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_options() {
        let input = "-io -u --export";
        let expected_options = CompilerOptions {
            io_only: true,
            optimize: true,
            export: true,
            update: true,
            wire_dot_out: false,
            illegal_states_out: false,
            wire_cross_out: false,
            print_after_all: false,
            print_before_backend: false,
            passes: None,
        };
        let options = CompilerOptions::parse(input);

        assert_eq!(options, expected_options);
    }
}
