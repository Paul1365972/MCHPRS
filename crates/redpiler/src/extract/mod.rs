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
        if let Some(&node) = nodes_by_position.get(&tick.pos) {
            graph[node].state.pending_tick = true;
        }
    }
    links::add_links(&mut graph, world, &nodes_by_position);
    graph
}
