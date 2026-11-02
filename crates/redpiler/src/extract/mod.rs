mod links;
mod nodes;

use crate::graph::{Graph, PendingTick, QueuedTick};
use crate::ports::{Ports, Role};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_commands::flag::{Flag, FlagValues};
use mchprs_redstone::tick_queue::TickEntry;
use std::fmt;

pub const IO_ONLY: Flag = Flag::switch(
    "io-only",
    Some('i'),
    "Only send block updates of inputs and outputs",
);

pub const WIRE_DOT_OUT: Flag =
    Flag::switch("wire-dot-out", Some('d'), "Treat redstone dots as outputs");

pub const ILLEGAL_STATES_OUT: Flag = Flag::switch(
    "illegal-states-out",
    Some('l'),
    "Treat wires in illegal states as outputs",
);

pub const WIRE_CROSS_OUT: Flag = Flag::switch(
    "wire-cross-out",
    Some('c'),
    "Treat redstone crosses as outputs",
);

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExtractOptions {
    pub ports: Ports,
    pub ports_only: bool,
    pub keeps_dust: bool,
}

impl ExtractOptions {
    pub fn from_flags(flags: &FlagValues) -> ExtractOptions {
        let mut ports = Ports::conventional();
        if flags.is_on(&WIRE_DOT_OUT) {
            ports = ports.with_wire_dots();
        }
        if flags.is_on(&ILLEGAL_STATES_OUT) {
            ports = ports.with_illegal_wires();
        }
        if flags.is_on(&WIRE_CROSS_OUT) {
            ports = ports.with_wire_crosses();
        }
        ExtractOptions {
            ports,
            ports_only: flags.is_on(&IO_ONLY),
            keeps_dust: true,
        }
    }

    pub fn observes(&self, port: Option<Role>) -> bool {
        port.is_some() || !self.ports_only
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ExtractError {
    NoComponent(BlockPos),
    NotDrivable(BlockPos),
}

impl fmt::Display for ExtractError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ExtractError::NoComponent(position) => {
                write!(f, "no component at declared port {position}")
            }
            ExtractError::NotDrivable(position) => {
                write!(f, "declared input at {position} can't be driven")
            }
        }
    }
}

impl std::error::Error for ExtractError {}

pub fn extract(
    world: &impl BlockView,
    region: Region,
    ticks: &[TickEntry],
    options: &ExtractOptions,
) -> Result<Graph, ExtractError> {
    let mut graph = Graph::default();
    let nodes_by_position = nodes::add_nodes(&mut graph, world, region, options);

    for (position, role) in options.ports.declared() {
        let Some(&node) = nodes_by_position.get(&position) else {
            return Err(ExtractError::NoComponent(position));
        };
        if role == Role::Input && graph[node].kind.drive().is_none() {
            return Err(ExtractError::NotDrivable(position));
        }
    }

    for (order, tick) in ticks.iter().enumerate() {
        if !tick.is_for(world.block(tick.position)) {
            continue;
        }
        let Some(&node) = nodes_by_position.get(&tick.position) else {
            continue;
        };
        let node = &mut graph[node];
        if !node.can_tick() {
            continue;
        }
        // A tick due in this tick but not yet run runs in the next one
        node.pending_tick = Some(QueuedTick {
            tick: PendingTick {
                delay: tick.ticks_left.max(1),
                priority: tick.priority,
            },
            order,
        });
    }

    links::add_links(&mut graph, world, &nodes_by_position);
    Ok(graph)
}
