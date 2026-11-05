mod opt;

use crate::graph::Graph;
use mchprs_commands::flag::Flag;
use std::ops::ControlFlow;
use tracing::trace;

pub use opt::coalesce::Coalesce;
pub use opt::constant_fold::ConstantFold;
pub use opt::dedup_links::DedupLinks;
pub use opt::prune_orphans::PruneOrphans;
pub use opt::unify_constants::UnifyConstants;
pub use opt::unreachable_output::UnreachableOutput;

pub const OPTIMIZE: Flag = Flag::switch(
    "optimize",
    Some('o'),
    "Enable Redpiler optimizations, which can break builds",
);

const ALL: [&dyn Pass; 6] = [
    &ConstantFold,
    &UnifyConstants,
    &DedupLinks,
    &UnreachableOutput,
    &Coalesce,
    &PruneOrphans,
];

pub trait Pass: Sync {
    fn run(&self, graph: &mut Graph);

    fn status_message(&self) -> &'static str;

    fn name(&self) -> &'static str;
}

pub fn named(name: &str) -> Option<&'static dyn Pass> {
    ALL.into_iter().find(|pass| pass.name() == name)
}

pub fn run_passes(
    passes: &[&dyn Pass],
    graph: &mut Graph,
    after_each: &mut dyn FnMut(&dyn Pass, &Graph) -> ControlFlow<()>,
) -> ControlFlow<()> {
    for &pass in passes {
        trace!("Running pass: {}", pass.name());
        pass.run(graph);
        trace!(
            "{} nodes and {} links remain",
            graph.node_count(),
            graph.edge_count()
        );
        after_each(pass, graph)?;
    }
    ControlFlow::Continue(())
}
