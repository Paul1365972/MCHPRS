use crate::engine::DirectEngine;
use mchprs_commands::flag::{Flag, FlagValues};
use mchprs_redpiler::backend::{Backend, BuildError, CompilePlan};
use mchprs_redpiler::circuit::Circuit;
use mchprs_redpiler::compile::{BuildContext, Stage};
use mchprs_redpiler::engine::Engine;
use mchprs_redpiler::extract::{
    ExtractOptions, LinkOrder, ILLEGAL_STATES_OUT, IO_ONLY, WIRE_CROSS_OUT, WIRE_DOT_OUT,
};
use mchprs_redpiler::passes::{
    Coalesce, ConstantFold, DedupLinks, Pass, PruneOrphans, UnreachableOutput, OPTIMIZE,
};

const REORDER: Flag = Flag::switch(
    "reorder",
    Some('r'),
    "Reorder updates within a tick for speed",
);

static FLAGS: [Flag; 6] = [
    OPTIMIZE,
    REORDER,
    IO_ONLY,
    WIRE_DOT_OUT,
    ILLEGAL_STATES_OUT,
    WIRE_CROSS_OUT,
];

const OPTIMIZING_PASSES: [&dyn Pass; 4] = [
    &DedupLinks,
    &ConstantFold,
    &UnreachableOutput,
    &PruneOrphans,
];

const REORDERING_PASSES: [&dyn Pass; 5] = [
    &DedupLinks,
    &ConstantFold,
    &UnreachableOutput,
    &Coalesce,
    &PruneOrphans,
];

#[derive(Debug, Clone, Copy, Default)]
pub struct DirectBackend;

impl Backend for DirectBackend {
    fn name(&self) -> &'static str {
        "direct"
    }

    fn flags(&self) -> &'static [Flag] {
        &FLAGS
    }

    fn configure(&self, flags: &FlagValues) -> Result<CompilePlan, String> {
        let optimize = flags.is_on(&OPTIMIZE);
        let reorder = flags.is_on(&REORDER);
        let passes = if !optimize {
            Vec::new()
        } else if reorder {
            REORDERING_PASSES.to_vec()
        } else {
            OPTIMIZING_PASSES.to_vec()
        };
        Ok(CompilePlan {
            extract: ExtractOptions {
                keeps_dust: !optimize,
                link_order: if reorder {
                    LinkOrder::TypeFirst
                } else {
                    LinkOrder::NearestFirst
                },
                ..ExtractOptions::from_flags(flags)
            },
            passes,
        })
    }

    fn build(
        &self,
        circuit: &Circuit,
        _: &FlagValues,
        context: &mut BuildContext<'_>,
    ) -> Result<Box<dyn Engine>, BuildError> {
        context.report(Stage::Building {
            step: "Building the engine",
            completed: 0,
            total: 1,
        })?;
        Ok(Box::new(DirectEngine::new(circuit)?))
    }
}
