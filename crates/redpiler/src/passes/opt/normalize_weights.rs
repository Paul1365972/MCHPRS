//! # [`NormalizeWeights`]
//!
//! A binary reader only tests whether any input delivers a signal, and a binary source delivers
//! one through every link that survived [`ClampWeights`]. Such links get a canonical weight, so
//! that [`DedupLinks`] and [`Coalesce`] match links that only differ in wire length.
//!
//! [`ClampWeights`]: super::clamp_weights::ClampWeights
//! [`DedupLinks`]: super::dedup_links::DedupLinks
//! [`Coalesce`]: super::coalesce::Coalesce

use crate::compile_graph::{CompileGraph, Direction, NodeIdx};
use crate::passes::{AnalysisInfos, Pass};
use crate::{CompilerInput, CompilerOptions};
use mchprs_world::World;
use tracing::trace;

const BINARY_LINK_WEIGHT: u8 = 0;

pub struct NormalizeWeights;

impl<W: World> Pass<W> for NormalizeWeights {
    fn run_pass(
        &self,
        graph: &mut CompileGraph,
        _: &CompilerOptions,
        _: &CompilerInput<'_, W>,
        _: &mut AnalysisInfos,
    ) {
        let mut num_normalized = 0;
        for i in 0..graph.node_bound() {
            let idx = NodeIdx::new(i);
            if !graph.contains_node(idx) || graph[idx].ty.reads_signal_strength() {
                continue;
            }
            let mut incoming = graph.neighbors(idx, Direction::Incoming).detach();
            while let Some((edge, source)) = incoming.next(graph) {
                if graph[source].ty.outputs_signal_strength() {
                    continue;
                }
                let link = &mut graph[edge];
                if link.ss != BINARY_LINK_WEIGHT {
                    link.ss = BINARY_LINK_WEIGHT;
                    num_normalized += 1;
                }
            }
        }
        trace!("Normalized {} link weights", num_normalized);
    }

    fn status_message(&self) -> &'static str {
        "Normalizing link weights"
    }

    fn driver_key(&self) -> &'static str {
        "normalize-weights"
    }
}
