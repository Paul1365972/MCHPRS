mod opt;

use crate::compile::CompileOptions;
use crate::graph::Graph;
use crate::ports::Observed;
use opt::clamp_weights::ClampWeights;
use opt::coalesce::Coalesce;
use opt::constant_fold::ConstantFold;
use opt::dedup_links::DedupLinks;
use opt::prune_orphans::PruneOrphans;
use opt::unreachable_output::UnreachableOutput;
use std::ops::ControlFlow;
use tracing::trace;

const ALL: [&dyn Pass; 6] = [
    &ClampWeights,
    &DedupLinks,
    &ConstantFold,
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

pub fn standard(options: &CompileOptions) -> Vec<&'static dyn Pass> {
    let mut passes: Vec<&'static dyn Pass> = vec![&ClampWeights];
    if options.optimize {
        passes.extend([&DedupLinks as &dyn Pass, &ConstantFold, &UnreachableOutput]);
        if options.reorder {
            passes.push(&Coalesce);
        }
        if options.observed == Observed::Ports {
            passes.push(&PruneOrphans);
        }
    }
    passes
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
