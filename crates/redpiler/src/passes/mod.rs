mod opt;

use crate::graph::Graph;
use mchprs_commands::flag::Flag;

pub use opt::coalesce::Coalesce;
pub use opt::dedup_links::DedupLinks;
pub use opt::fold::Fold;
pub use opt::prune_orphans::PruneOrphans;
pub use opt::unify_constants::UnifyConstants;
pub use opt::unreachable_output::UnreachableOutput;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UpdateOrder {
    Kept,
    Free,
}

pub const OPTIMIZE: Flag = Flag::switch(
    "optimize",
    Some('o'),
    "Optimize the circuit without changing its behavior",
);

const ALL: [&dyn Pass; 8] = [
    &DedupLinks(UpdateOrder::Kept),
    &DedupLinks(UpdateOrder::Free),
    &Fold,
    &UnreachableOutput,
    &UnifyConstants,
    &Coalesce(UpdateOrder::Kept),
    &Coalesce(UpdateOrder::Free),
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
