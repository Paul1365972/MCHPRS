use super::{ExtractError, ExtractOptions};
use crate::graph::{Graph, Node, NodeIdx, NodeKind, NodeState};
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::signal_strength::{SignalStrength, UncappedSignalStrength};
use mchprs_blocks::BlockPos;
use mchprs_redstone::{self, comparator, noteblock};
use rustc_hash::FxHashMap;
use smallvec::smallvec;

pub fn add_nodes(
    graph: &mut Graph,
    world: &impl BlockView,
    region: Region,
    options: &ExtractOptions,
) -> Result<FxHashMap<BlockPos, NodeIdx>, ExtractError> {
    let mut nodes_by_position = FxHashMap::default();
    for pos in world
        .occupied_regions(region)
        .into_iter()
        .flat_map(Region::positions)
    {
        if let Some(node) = add_node(graph, world, pos, options)? {
            nodes_by_position.insert(pos, node);
        }
    }
    Ok(nodes_by_position)
}

fn add_node(
    graph: &mut Graph,
    world: &impl BlockView,
    pos: BlockPos,
    options: &ExtractOptions,
) -> Result<Option<NodeIdx>, ExtractError> {
    let block = world.block(pos);
    let Some((kind, state)) = identify_block(block, pos, world)? else {
        return Ok(None);
    };
    let port = options.ports.role(pos, block, &kind);
    if kind == NodeKind::Wire && port.is_none() && !options.keeps_dust {
        return Ok(None);
    }
    Ok(Some(graph.add_node(Node {
        kind,
        blocks: smallvec![(pos, block.get_id())],
        names: Vec::new(),
        state,
        pending_tick: None,
        port,
        observed: options.observes(port),
    })))
}

fn identify_block(
    block: Block,
    pos: BlockPos,
    world: &impl BlockView,
) -> Result<Option<(NodeKind, NodeState)>, ExtractError> {
    if let Some(powered) = block.pressure_plate_powered() {
        return Ok(Some((
            NodeKind::PressurePlate,
            NodeState::from_powered(powered),
        )));
    }
    let (kind, state) = match block {
        Block::Repeater(repeater) => (
            NodeKind::Repeater {
                delay: repeater.delay,
                prioritized: mchprs_redstone::diode_should_be_prioritized(
                    world,
                    pos,
                    repeater.facing,
                ),
            },
            NodeState::repeater(repeater.powered, repeater.locked),
        ),
        Block::Comparator(comparator) => (
            NodeKind::Comparator {
                mode: comparator.mode,
                far_input: comparator::get_far_input(world, pos, comparator.facing)
                    .map(|power| within_fifteen(power, pos))
                    .transpose()?,
                prioritized: mchprs_redstone::diode_should_be_prioritized(
                    world,
                    pos,
                    comparator.facing,
                ),
            },
            NodeState::from_power(match world.block_entity(pos) {
                Some(BlockEntity::Comparator { output_strength }) => {
                    within_fifteen(*output_strength, pos)?
                }
                _ => SignalStrength::Zero,
            }),
        ),
        Block::RedstoneTorch { lit, .. } | Block::RedstoneWallTorch { lit, .. } => {
            (NodeKind::Torch, NodeState::from_powered(lit))
        }
        Block::RedstoneWire(wire) => (NodeKind::Wire, NodeState::from_power(wire.power)),
        _ if let Some(button) = block.button() => (
            NodeKind::Button {
                press_ticks: mchprs_redstone::button_press_ticks(block),
            },
            NodeState::from_powered(button.powered),
        ),
        Block::RedstoneLamp { lit } => (NodeKind::Lamp, NodeState::from_powered(lit)),
        Block::Lever { powered, .. } => (NodeKind::Lever, NodeState::from_powered(powered)),
        Block::IronTrapdoor { powered, .. } => {
            (NodeKind::Trapdoor, NodeState::from_powered(powered))
        }
        Block::RedstoneBlock => (
            NodeKind::Constant,
            NodeState::from_power(SignalStrength::Fifteen),
        ),
        Block::NoteBlock { note, powered, .. } if noteblock::is_noteblock_unblocked(world, pos) => {
            let instrument = noteblock::get_noteblock_instrument(world, pos);
            (
                NodeKind::NoteBlock { instrument, note },
                NodeState::from_powered(powered),
            )
        }
        _ => {
            let Some(power) = comparator::analog_output(world, pos, block) else {
                return Ok(None);
            };
            (
                NodeKind::Constant,
                NodeState::from_power(within_fifteen(power, pos)?),
            )
        }
    };
    Ok(Some((kind, state)))
}

fn within_fifteen(
    strength: UncappedSignalStrength,
    pos: BlockPos,
) -> Result<SignalStrength, ExtractError> {
    SignalStrength::try_from(strength).map_err(|_| ExtractError::AboveFifteen(pos))
}
