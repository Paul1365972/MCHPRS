pub mod coalesce;
pub mod fold;
pub mod prune_orphans;
pub mod sort_links;
pub mod unify_constants;

use crate::graph::Graph;
use std::fmt;

pub trait Pass: fmt::Debug + Sync {
    fn name(&self) -> &'static str;

    fn run(&self, graph: &mut Graph);
}

pub fn all() -> Vec<Box<dyn Pass>> {
    vec![
        Box::new(fold::Fold),
        Box::new(unify_constants::UnifyConstants),
        Box::new(prune_orphans::PruneOrphans),
        Box::new(sort_links::SortLinks),
        Box::new(coalesce::Coalesce),
    ]
}
