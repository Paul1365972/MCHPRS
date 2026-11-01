mod links;
mod nodes;

use crate::graph::{Graph, NodeKind, PendingTick, QueuedTick};
use crate::ports::Role;
use crate::CompileOptions;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_redstone::tick_queue::TickEntry;
use std::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ExtractError {
    NoComponent(BlockPos),
    NotDrivable(BlockPos),
}

impl fmt::Display for ExtractError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ExtractError::NoComponent(pos) => {
                write!(f, "no component at declared port {pos}")
            }
            ExtractError::NotDrivable(pos) => {
                write!(f, "declared input at {pos} can't be driven")
            }
        }
    }
}

impl std::error::Error for ExtractError {}

pub fn extract(
    world: &impl BlockView,
    region: Region,
    ticks: &[TickEntry],
    options: &CompileOptions,
) -> Result<Graph, ExtractError> {
    let mut graph = Graph::default();
    let nodes_by_position = nodes::add_nodes(&mut graph, world, region, options);

    for (pos, role) in options.ports.declared() {
        let Some(&node) = nodes_by_position.get(&pos) else {
            return Err(ExtractError::NoComponent(pos));
        };
        if role == Role::Input && !graph[node].kind.is_drivable() {
            return Err(ExtractError::NotDrivable(pos));
        }
    }

    for (order, tick) in ticks.iter().enumerate() {
        if !tick.is_for(world.block(tick.pos)) {
            continue;
        }
        let Some(&node) = nodes_by_position.get(&tick.pos) else {
            continue;
        };
        let node = &mut graph[node];
        if node.is_input() && node.kind != NodeKind::Button {
            continue;
        }
        node.state.pending_tick = Some(QueuedTick {
            tick: PendingTick {
                delay: tick.ticks_left,
                priority: tick.priority,
            },
            order,
        });
    }

    links::add_links(&mut graph, world, &nodes_by_position);
    Ok(graph)
}
