pub mod builder;

use super::rng::Rng;
use super::scenario_generator::{random_steps, WARM_UP_TICKS};
use crate::input::World;
use crate::oracle::Step;
use crate::setup::Setup;
use crate::Error;
use anyhow::{anyhow, Context};
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::BlockPos;
use mchprs_redpiler::backend::GraphContext;
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::graph::Graph;

pub fn warm_up(world: &mut World, rng: &mut Rng) -> Result<Graph, Error> {
    let circuit = Circuit::new(extracted_graph(world)?)?;
    let ticks = rng.within(1..=WARM_UP_TICKS);
    for step in random_steps(&circuit, rng, ticks) {
        match step {
            Step::Drive(node, input) => {
                let pos = position(&circuit, node);
                world
                    .drive(pos, input)
                    .map_err(|error| anyhow!("the block at {pos} {error}"))?;
            }
            Step::Tick => world.tick(),
        }
    }
    world.take_sounds();
    let mut graph = extracted_graph(world)?;
    for node in graph.node_indices().collect::<Vec<_>>() {
        let (pos, _) = graph[node].blocks[0];
        graph[node].state.plate_touched = world.is_standing_on(pos);
    }
    Ok(graph)
}

fn extracted_graph(world: &World) -> Result<Graph, Error> {
    let graph = Setup::reference().world_graph(world, &mut GraphContext::silent());
    graph.context("the world does not compile")
}

pub fn position(circuit: &Circuit, node: NodeId) -> BlockPos {
    let (pos, _) = circuit.node(node).blocks[0];
    pos
}

pub fn describe(world: &World) -> String {
    let mut lines = Vec::new();
    for pos in world.blocks().region().positions() {
        if world.block(pos) != Block::Air {
            lines.push(format!("{pos} {}", describe_block(world, pos)));
        }
        if world.is_standing_on(pos) {
            lines.push(format!("{pos} is touched"));
        }
    }
    for tick in world.tick_entries() {
        lines.push(format!(
            "{} ticks in {} with priority {}",
            tick.pos, tick.ticks_left, tick.priority
        ));
    }
    lines.join("\n")
}

pub fn describe_block(blocks: &impl BlockView, pos: BlockPos) -> String {
    let block = blocks.block(pos);
    match blocks.block_entity(pos) {
        Some(BlockEntity::Comparator { output_strength }) => {
            format!("{block} with output strength {}", output_strength.get())
        }
        None if matches!(block, Block::Comparator(_)) => format!("{block} with output strength 0"),
        Some(BlockEntity::Container { inventory, ty }) => {
            format!(
                "{block} with signal strength {}",
                ty.signal_strength(inventory).get()
            )
        }
        _ => block.to_string(),
    }
}
