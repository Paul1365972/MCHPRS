use super::{ExtractError, ExtractOptions};
use crate::block_node::component_of;
use crate::graph::{Graph, Node, NodeIdx, NodeKind};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_simulation::ports::Role;
use rustc_hash::{FxHashMap, FxHashSet};
use smallvec::smallvec;

pub fn add_nodes(
    graph: &mut Graph,
    world: &impl BlockView,
    region: Region,
    declared: &FxHashMap<BlockPos, Role>,
    observed: &FxHashSet<BlockPos>,
    options: &ExtractOptions,
) -> Result<FxHashMap<BlockPos, NodeIdx>, ExtractError> {
    let mut nodes_by_position = FxHashMap::default();
    for pos in world.occupied_positions(region) {
        if let Some(node) = add_node(graph, world, pos, declared, observed, options)? {
            nodes_by_position.insert(pos, node);
        }
    }
    Ok(nodes_by_position)
}

fn add_node(
    graph: &mut Graph,
    world: &impl BlockView,
    pos: BlockPos,
    declared: &FxHashMap<BlockPos, Role>,
    observed: &FxHashSet<BlockPos>,
    options: &ExtractOptions,
) -> Result<Option<NodeIdx>, ExtractError> {
    let block = world.block(pos);
    let Some((kind, state)) = component_of(world, pos, block)? else {
        return Ok(None);
    };
    let port = match declared.get(&pos) {
        Some(&role) => Some(role),
        None => options.wire_outputs.role(block, kind),
    };
    let observed = observed.contains(&pos);
    if kind == NodeKind::Wire && port.is_none() && !observed && !options.keeps_dust {
        return Ok(None);
    }
    Ok(Some(graph.add_node(Node {
        kind,
        blocks: smallvec![(pos, block.get_id())],
        names: Vec::new(),
        state,
        pending_tick: None,
        port,
        observed: observed || options.observes(port),
    })))
}
