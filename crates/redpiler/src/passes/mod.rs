mod opt;

use crate::graph::Graph;
use mchprs_commands::flag::Flag;

pub use opt::coalesce::Coalesce;
pub use opt::dedup_links::DedupLinks;
pub use opt::fold::Fold;
pub use opt::prune_orphans::PruneOrphans;
pub use opt::unreachable_output::UnreachableOutput;

pub const OPTIMIZE: Flag = Flag::switch(
    "optimize",
    Some('o'),
    "Enable Redpiler optimizations, which can break builds",
);

const ALL: [&dyn Pass; 5] = [
    &DedupLinks,
    &Fold,
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
