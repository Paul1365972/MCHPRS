pub mod backend;
pub mod extract;
pub mod graph;
pub mod passes;
pub mod ports;
pub mod ril;
pub mod string_replacer;
pub mod task_monitor;

use crate::extract::ExtractError;
use crate::ports::{Observed, Ports};
use backend::{BackendDispatcher, JITBackend};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_redstone::tick_queue::TickEntry;
use mchprs_redstone::RedstoneWorld;
use passes::PassRegistry;
use ril::DumpGraph;
use std::ops::ControlFlow;
use std::sync::Arc;
use std::time::Instant;
use tracing::{debug, error, trace, warn};

pub use task_monitor::TaskMonitor;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CompileOptions {
    pub ports: Ports,
    pub observed: Observed,
    pub optimize: bool,
    pub reorder: bool,
}

impl Default for CompileOptions {
    fn default() -> Self {
        CompileOptions {
            ports: Ports::conventional(),
            observed: Observed::Everything,
            optimize: false,
            reorder: false,
        }
    }
}

fn block_powered_mut(block: &mut Block) -> Option<&mut bool> {
    Some(match block {
        Block::Comparator(comparator) => &mut comparator.powered,
        Block::RedstoneTorch { lit } => lit,
        Block::RedstoneWallTorch { lit, .. } => lit,
        Block::Repeater(repeater) => &mut repeater.powered,
        Block::Lever { powered, .. } => powered,
        Block::StoneButton { powered, .. } => powered,
        Block::RedstoneLamp { lit } => lit,
        Block::IronTrapdoor { powered, .. } => powered,
        Block::NoteBlock { powered, .. } => powered,
        _ => return block.get_pressure_plate_powered(),
    })
}

#[derive(Default, PartialEq, Eq, Debug, Clone)]
pub struct CompilerOptions {
    /// Enable optimization passes which may significantly increase compile times.
    pub optimize: bool,
    /// Let optimizations reorder updates within a tick
    pub reorder: bool,
    /// Only flush lamp, button, lever, pressure plate, or trapdoor updates.
    pub io_only: bool,
    /// Update all blocks in the input region after reset.
    pub update: bool,
    /// Export a dot file of the graph after backend compile (backend dependent)
    pub export_dot_graph: bool,
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
    /// The backend variant to be used after compilation
    pub backend_variant: BackendVariant,
    /// A comma separated list of passes to run. This can only be used by the rilc driver.
    pub passes: Option<String>,
}

#[derive(Debug, Default, PartialEq, Eq, Clone, Copy)]
pub enum BackendVariant {
    #[default]
    Direct,
}

impl CompilerOptions {
    pub fn compile_options(&self) -> CompileOptions {
        let mut ports = Ports::conventional();
        if self.wire_dot_out {
            ports = ports.with_wire_dots();
        }
        if self.wire_cross_out {
            ports = ports.with_wire_crosses();
        }
        if self.illegal_states_out {
            ports = ports.with_illegal_wires();
        }
        let observed = if self.io_only {
            Observed::Ports
        } else if self.optimize {
            Observed::Components
        } else {
            Observed::Everything
        };
        CompileOptions {
            ports,
            observed,
            optimize: self.optimize,
            reorder: self.reorder,
        }
    }

    fn parse_option(&mut self, option: &str) {
        if option.starts_with("--") {
            if let Some(passes_str) = option.strip_prefix("--passes=") {
                self.passes = Some(passes_str.to_owned());
                return;
            }

            match option {
                "--optimize" => self.optimize = true,
                "--reorder" => self.reorder = true,
                "--io-only" => self.io_only = true,
                "--update" => self.update = true,
                "--export-dot" => self.export_dot_graph = true,
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
                    "r" => self.reorder = true,
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

#[derive(Default)]
pub struct Compiler {
    is_active: bool,
    backend: Option<BackendDispatcher>,
    options: CompilerOptions,
}

impl Compiler {
    pub fn is_active(&self) -> bool {
        self.is_active
    }

    pub fn current_flags(&self) -> Option<&CompilerOptions> {
        match self.is_active {
            true => Some(&self.options),
            false => None,
        }
    }

    /// Switches the currently active backend to the one specified by `backend`.
    /// Requires recompilation to take effect.
    pub fn use_backend(&mut self, backend: BackendDispatcher) {
        self.backend = Some(backend);
    }

    pub fn compile<W: BlockView + 'static>(
        &mut self,
        world: &W,
        region: Region,
        options: CompilerOptions,
        ticks: Vec<TickEntry>,
        monitor: Arc<TaskMonitor>,
    ) -> Result<(), ExtractError> {
        debug!("Starting compile");
        let start = Instant::now();
        let compile_options = options.compile_options();

        monitor.set_message("Extracting circuit".to_string());
        let mut graph = extract::extract(world, region, &ticks, &compile_options)?;
        let registry = PassRegistry::default();
        let pass_pipeline = passes::build_pass_pipeline(&registry, &compile_options);
        // Add one for the backend compile step
        monitor.set_max_progress(pass_pipeline.len() + 1);
        let flow = pass_pipeline.run_passes(&mut graph, &mut |pass, graph| {
            monitor.inc_progress();
            monitor.set_message(pass.status_message().to_string());
            if options.print_after_all {
                debug!("Printing circuit after pass: {}", pass.debug_name());
                graph.dump();
            }
            match monitor.cancelled() {
                true => ControlFlow::Break(()),
                false => ControlFlow::Continue(()),
            }
        });
        if flow.is_break() {
            return Ok(());
        }
        if options.print_before_backend {
            debug!("Printing circuit before backend compile:");
            graph.dump();
        }

        let replace_backend = match self.backend {
            Some(BackendDispatcher::DirectBackend(_)) => {
                options.backend_variant != BackendVariant::Direct
            }
            None => true,
        };
        if replace_backend {
            debug!("Switching backend to {:?}", options.backend_variant);
            let backend = match options.backend_variant {
                BackendVariant::Direct => BackendDispatcher::DirectBackend(Default::default()),
            };
            self.use_backend(backend);
        }

        if let Some(backend) = &mut self.backend {
            trace!("Compiling backend");
            monitor.set_message("Compiling backend".to_string());
            let start = Instant::now();

            backend.compile(graph, &options, monitor.clone());

            monitor.inc_progress();
            trace!("Backend compiled in {:?}", start.elapsed());
        } else {
            error!("Cannot compile without backend variant selected");
        }

        self.options = options;
        self.is_active = true;
        debug!("Compile completed in {:?}", start.elapsed());
        Ok(())
    }

    pub fn reset<W: RedstoneWorld>(&mut self, world: &mut W, region: Region) {
        if self.is_active {
            self.is_active = false;
            if let Some(backend) = &mut self.backend {
                backend.reset(world)
            }
        }

        if self.options.update {
            for pos in world
                .occupied_regions(region)
                .into_iter()
                .flat_map(Region::positions)
            {
                let block = world.block(pos);
                mchprs_redstone::update(block, world, pos);
            }
        }
        self.options = Default::default();
    }

    fn backend(&mut self) -> &mut BackendDispatcher {
        assert!(
            self.is_active,
            "tried to get redpiler backend when inactive"
        );
        if let Some(backend) = &mut self.backend {
            backend
        } else {
            panic!("redpiler is active but is missing backend");
        }
    }

    pub fn tick(&mut self) {
        self.backend().tick();
    }

    pub fn tickn(&mut self, ticks: u64) {
        self.backend().tickn(ticks);
    }

    pub fn on_use_block(&mut self, pos: BlockPos) {
        self.backend().on_use_block(pos);
    }

    pub fn set_pressure_plate(&mut self, pos: BlockPos, powered: bool) {
        self.backend().set_pressure_plate(pos, powered);
    }

    pub fn flush<W: RedstoneWorld>(&mut self, world: &mut W) {
        self.backend().flush(world);
    }

    pub fn flush_all<W: RedstoneWorld>(&mut self, world: &mut W) {
        self.backend().flush_all(world);
    }

    pub fn has_pending_ticks(&mut self) -> bool {
        self.backend().has_pending_ticks()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_options() {
        let input = "-io -u";
        let expected_options = CompilerOptions {
            io_only: true,
            optimize: true,
            reorder: false,
            update: true,
            export_dot_graph: false,
            wire_dot_out: false,
            illegal_states_out: false,
            wire_cross_out: false,
            print_after_all: false,
            print_before_backend: false,
            backend_variant: BackendVariant::default(),
            passes: None,
        };
        let options = CompilerOptions::parse(input);

        assert_eq!(options, expected_options);
    }
}
