use super::ExtractOptions;
use crate::graph::{Graph, Node, NodeIdx, NodeKind, NodeState};
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::BlockPos;
use mchprs_redstone::{self, comparator, noteblock};
use rustc_hash::FxHashMap;
use smallvec::smallvec;

pub fn add_nodes(
    graph: &mut Graph,
    world: &impl BlockView,
    region: Region,
    options: &ExtractOptions,
) -> FxHashMap<BlockPos, NodeIdx> {
    let mut nodes_by_position = FxHashMap::default();
    for pos in world
        .occupied_regions(region)
        .into_iter()
        .flat_map(Region::positions)
    {
        if let Some(node) = add_node(graph, world, pos, options) {
            nodes_by_position.insert(pos, node);
        }
    }
    nodes_by_position
}

fn add_node(
    graph: &mut Graph,
    world: &impl BlockView,
    pos: BlockPos,
    options: &ExtractOptions,
) -> Option<NodeIdx> {
    let block = world.block(pos);
    let (kind, state) = identify_block(block, pos, world)?;
    let port = options.ports.role(pos, block, &kind);
    if kind == NodeKind::Wire && port.is_none() && !options.keeps_dust {
        return None;
    }
    Some(graph.add_node(Node {
        kind,
        blocks: smallvec![(pos, block.get_id())],
        name: None,
        state,
        pending_tick: None,
        port,
        observed: options.observes(port),
    }))
}

fn identify_block(
    block: Block,
    pos: BlockPos,
    world: &impl BlockView,
) -> Option<(NodeKind, NodeState)> {
    if let Some(powered) = block.clone().get_pressure_plate_powered() {
        return Some((NodeKind::PressurePlate, NodeState::from_powered(*powered)));
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
                far_input: comparator::get_far_input(world, pos, comparator.facing).map(|power| {
                    SignalStrength::try_from(power).expect("invalid comparator far input")
                }),
                prioritized: mchprs_redstone::diode_should_be_prioritized(
                    world,
                    pos,
                    comparator.facing,
                ),
            },
            NodeState::from_power(match world.block_entity(pos) {
                Some(BlockEntity::Comparator { output_strength }) => *output_strength,
                _ => SignalStrength::Zero,
            }),
        ),
        Block::RedstoneTorch { lit, .. } | Block::RedstoneWallTorch { lit, .. } => {
            (NodeKind::Torch, NodeState::from_powered(lit))
        }
        Block::RedstoneWire(wire) => (
            NodeKind::Wire,
            NodeState::from_power(
                SignalStrength::try_from(wire.power).expect("invalid wire power"),
            ),
        ),
        Block::StoneButton { powered, .. } => (NodeKind::Button, NodeState::from_powered(powered)),
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
            let power = comparator::analog_output(block, world, pos)?;
            (
                NodeKind::Constant,
                NodeState::from_power(
                    SignalStrength::try_from(power).expect("invalid comparator override"),
                ),
            )
        }
    };
    Some((kind, state))
}
