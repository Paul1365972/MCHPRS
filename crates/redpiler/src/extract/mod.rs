//! Turns the redstone components in a region of a world into a [`CompileGraph`].
//! Every component in the region becomes a node. Links lead from the components that power a
//! node to it, through dust where needed, weighted by the dust distance. Sources outside the
//! region are not linked.

mod links;
mod nodes;

use crate::compile_graph::CompileGraph;
use crate::CompilerOptions;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::region::Region;
use mchprs_redstone::tick_queue::TickEntry;

pub fn extract(
    world: &impl BlockView,
    region: Region,
    ticks: &[TickEntry],
    options: &CompilerOptions,
) -> CompileGraph {
    let mut graph = CompileGraph::default();
    let nodes_by_position = nodes::add_nodes(&mut graph, world, region, options);
    for tick in ticks {
        if let Some(&node) = nodes_by_position.get(&tick.position) {
            graph[node].state.pending_tick = true;
        }
    }
    links::add_links(&mut graph, world, &nodes_by_position);
    graph
}
