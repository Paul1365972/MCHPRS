use crate::engine::PhaseEngine;
use mchprs_redpiler::backend::{GraphBackend, GraphContext};
use mchprs_redpiler::circuit::Circuit;
use mchprs_redpiler::extract::ExtractOptions;
use mchprs_redpiler::graph_simulation;
use mchprs_redpiler::nodes::Nodes;
use mchprs_redpiler::passes::fold::Fold;
use mchprs_redpiler::passes::prune_orphans::PruneOrphans;
use mchprs_redpiler::passes::unify_constants::UnifyConstants;
use mchprs_redpiler::passes::Pass;
use mchprs_redpiler::ports::WireOutputs;
use mchprs_simulation::backend::{Backend, CompileContext, CompileError, CompileStep, Source};
use mchprs_simulation::layout::Layout;
use mchprs_simulation::{Blocks, Simulation};
use mchprs_syntax::flag::{Flag, FlagValues};

pub const OPTIMIZE: Flag = Flag::switch("optimize", Some('o'), "Optimize the circuit");

const IO_ONLY: Flag = Flag::switch(
    "io-only",
    Some('i'),
    "Only update the blocks of inputs, outputs and probed components",
);

#[derive(Debug, Clone, Copy, Default)]
pub struct PhaseBackend;

impl Backend for PhaseBackend {
    fn name(&self) -> &str {
        "phase"
    }

    fn flags(&self) -> &[Flag] {
        &[
            OPTIMIZE,
            IO_ONLY,
            WireOutputs::DOTS,
            WireOutputs::ILLEGAL_STATES,
            WireOutputs::CROSSES,
        ]
    }

    fn compile(
        &self,
        source: &Source<'_>,
        flags: &FlagValues,
        context: &mut CompileContext<'_>,
    ) -> Result<(Box<dyn Simulation<Blocks>>, Layout), CompileError> {
        graph_simulation::compile(self, source, flags, context)
    }
}

impl GraphBackend for PhaseBackend {
    fn extract_options(&self, flags: &FlagValues) -> ExtractOptions {
        ExtractOptions {
            wire_outputs: WireOutputs::from_flags(flags),
            ports_only: flags.is_on(&IO_ONLY),
            keeps_dust: !flags.is_on(&OPTIMIZE),
        }
    }

    fn passes(&self, flags: &FlagValues) -> Vec<Box<dyn Pass>> {
        if !flags.is_on(&OPTIMIZE) {
            return Vec::new();
        }
        vec![
            Box::new(Fold),
            Box::new(UnifyConstants),
            Box::new(PruneOrphans),
        ]
    }

    fn build(
        &self,
        circuit: &Circuit,
        _: &FlagValues,
        context: &mut GraphContext<'_>,
    ) -> Result<Box<dyn Simulation<Nodes>>, CompileError> {
        context.report(CompileStep::single("Building the engine"))?;
        Ok(Box::new(PhaseEngine::new(circuit)))
    }
}
