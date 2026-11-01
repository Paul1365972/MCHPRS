use crate::compile_graph::{CompileGraph, CompileNode, NodeIdx, NodeState, NodeType};
use crate::CompilerOptions;
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::BlockPos;
use mchprs_redstone::{self, comparator, noteblock, wire};
use rustc_hash::FxHashMap;
use smallvec::smallvec;

pub fn add_nodes(
    graph: &mut CompileGraph,
    world: &impl BlockView,
    region: Region,
    options: &CompilerOptions,
) -> FxHashMap<BlockPos, NodeIdx> {
    let mut nodes_by_position = FxHashMap::default();
    for position in world
        .occupied_regions(region)
        .into_iter()
        .flat_map(Region::positions)
    {
        if let Some(node) = add_node(graph, world, position, options) {
            nodes_by_position.insert(position, node);
        }
    }
    nodes_by_position
}

fn add_node(
    graph: &mut CompileGraph,
    world: &impl BlockView,
    position: BlockPos,
    options: &CompilerOptions,
) -> Option<NodeIdx> {
    let block = world.block(position);
    let (ty, state) = identify_block(block, position, world)?;

    let is_input = ty.is_normally_input();
    let is_output = ty.is_normally_output()
        || matches!(
            block,
            Block::RedstoneWire(wire) if
                options.wire_dot_out && wire::is_dot(wire) ||
                options.illegal_states_out && wire::is_illegal(wire) ||
                options.wire_cross_out && wire::is_cross(wire)
        );

    if options.optimize && ty == NodeType::Wire && !(is_input | is_output) {
        return None;
    }

    Some(graph.add_node(CompileNode {
        ty,
        block: smallvec![(position, block.get_id())],
        name: None,
        state,
        is_input,
        is_output,
    }))
}

fn identify_block(
    block: Block,
    position: BlockPos,
    world: &impl BlockView,
) -> Option<(NodeType, NodeState)> {
    if let Some(powered) = block.clone().get_pressure_plate_powered() {
        return Some((NodeType::PressurePlate, NodeState::from_powered(*powered)));
    }
    let (ty, state) = match block {
        Block::Repeater(repeater) => (
            NodeType::Repeater {
                delay: repeater.delay,
                prioritized: mchprs_redstone::diode_should_be_prioritized(
                    world,
                    position,
                    repeater.facing,
                ),
            },
            NodeState::repeater(repeater.powered, repeater.locked),
        ),
        Block::Comparator(comparator) => (
            NodeType::Comparator {
                mode: comparator.mode,
                far_input: comparator::get_far_input(world, position, comparator.facing).map(
                    |power| SignalStrength::try_from(power).expect("invalid comparator far input"),
                ),
                prioritized: mchprs_redstone::diode_should_be_prioritized(
                    world,
                    position,
                    comparator.facing,
                ),
            },
            NodeState::from_power(match world.block_entity(position) {
                Some(BlockEntity::Comparator { output_strength }) => *output_strength,
                _ => SignalStrength::Zero,
            }),
        ),
        Block::RedstoneTorch { lit, .. } | Block::RedstoneWallTorch { lit, .. } => {
            (NodeType::Torch, NodeState::from_powered(lit))
        }
        Block::RedstoneWire(wire) => (
            NodeType::Wire,
            NodeState::from_power(
                SignalStrength::try_from(wire.power).expect("invalid wire power"),
            ),
        ),
        Block::StoneButton { powered, .. } => (NodeType::Button, NodeState::from_powered(powered)),
        Block::RedstoneLamp { lit } => (NodeType::Lamp, NodeState::from_powered(lit)),
        Block::Lever { powered, .. } => (NodeType::Lever, NodeState::from_powered(powered)),
        Block::IronTrapdoor { powered, .. } => {
            (NodeType::Trapdoor, NodeState::from_powered(powered))
        }
        Block::RedstoneBlock => (
            NodeType::Constant,
            NodeState::from_power(SignalStrength::Fifteen),
        ),
        Block::NoteBlock { note, powered, .. }
            if noteblock::is_noteblock_unblocked(world, position) =>
        {
            let instrument = noteblock::get_noteblock_instrument(world, position);
            (
                NodeType::NoteBlock { instrument, note },
                NodeState::from_powered(powered),
            )
        }
        _ => {
            let power = comparator::analog_output(block, world, position)?;
            (
                NodeType::Constant,
                NodeState::from_power(
                    SignalStrength::try_from(power).expect("invalid comparator override"),
                ),
            )
        }
    };
    Some((ty, state))
}
