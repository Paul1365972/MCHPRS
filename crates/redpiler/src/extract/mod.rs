mod links;
mod nodes;

use crate::graph::{Graph, QueuedTick};
use crate::ports::WireOutputs;
use mchprs_blocks::BlockPos;
use mchprs_redstone::tick_queue::PendingTick;
use mchprs_simulation::backend::{PortError, Source};
use mchprs_simulation::ports::Role;
use rustc_hash::{FxHashMap, FxHashSet};

pub use crate::block_node::component_at;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExtractOptions {
    pub wire_outputs: WireOutputs,
    pub ports_only: bool,
    pub keeps_dust: bool,
}

impl ExtractOptions {
    pub fn observes(&self, port: Option<Role>) -> bool {
        port.is_some() || !self.ports_only
    }

    pub fn observe(&self, graph: &mut Graph) {
        let nodes: Vec<_> = graph.node_indices().collect();
        for node in nodes {
            graph[node].observed = self.observes(graph[node].port);
        }
    }
}

impl Default for ExtractOptions {
    fn default() -> ExtractOptions {
        ExtractOptions {
            wire_outputs: WireOutputs::default(),
            ports_only: false,
            keeps_dust: true,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ExtractError {
    #[error(transparent)]
    Port(#[from] PortError),
    #[error("signal strength above 15 at {0}")]
    AboveFifteen(BlockPos),
}

pub fn extract(source: &Source<'_>, options: &ExtractOptions) -> Result<Graph, ExtractError> {
    let world = source.blocks;
    let declared: FxHashMap<BlockPos, Role> = source.declared.iter().copied().collect();
    let observed: FxHashSet<BlockPos> = source.observed.iter().copied().collect();
    let mut graph = Graph::default();
    let nodes_by_position = nodes::add_nodes(
        &mut graph,
        &world,
        source.region,
        &declared,
        &observed,
        options,
    )?;

    for &(pos, role) in source.declared {
        let Some(&node) = nodes_by_position.get(&pos) else {
            return Err(PortError::NoComponent(pos).into());
        };
        if role == Role::Input && graph[node].kind.traits().drive.is_none() {
            return Err(PortError::NotDrivable(pos).into());
        }
    }

    for (order, tick) in source.ticks.iter().enumerate() {
        if !tick.is_for(world.block(tick.pos)) {
            continue;
        }
        let Some(&node) = nodes_by_position.get(&tick.pos) else {
            continue;
        };
        let node = &mut graph[node];
        if !node.can_tick() {
            continue;
        }
        node.pending_tick = Some(QueuedTick {
            tick: PendingTick {
                // A tick that is due but has not run runs in the next tick.
                delay: tick.ticks_left.max(1),
                priority: tick.priority,
            },
            order,
        });
    }

    links::add_links(&mut graph, &world, &nodes_by_position);
    Ok(graph)
}
