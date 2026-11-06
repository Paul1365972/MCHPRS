use crate::{attached, bool_to_ss, comparator, diode, wire, DIRECTIONS};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::{BlockFace, BlockPos};

/// Dust mutes itself while it computes its own power, as vanilla's `shouldSignal` does
#[derive(Clone, Copy, PartialEq)]
pub(crate) enum Dust {
    Emits,
    Muted,
}

pub(crate) fn is_signal_source(state: Block) -> bool {
    matches!(
        state,
        Block::RedstoneWire(_)
            | Block::Repeater(_)
            | Block::Comparator(_)
            | Block::RedstoneTorch { .. }
            | Block::RedstoneWallTorch { .. }
            | Block::RedstoneBlock
            | Block::Target { .. }
            | Block::TripwireHook { .. }
    ) || attached::attachment(state).is_some()
        || state.pressure_plate_powered().is_some()
}

/// Returns `true` if the given block provides either weak or strong power to the given side.
/// Note that `side` is the side of the block receiving power, not the side of the block providing power.
pub fn provides_weak_power(block: Block, side: BlockFace) -> bool {
    match block {
        Block::RedstoneTorch { .. } => side != BlockFace::Top,
        Block::RedstoneWallTorch { facing, .. } => facing.block_face() != side,
        Block::RedstoneBlock | Block::TripwireHook { .. } | Block::Target { .. } => true,
        Block::Repeater(repeater) => repeater.facing.block_face() == side,
        Block::Comparator(comparator) => comparator.facing.block_face() == side,
        _ => attached::attachment(block).is_some() || block.pressure_plate_powered().is_some(),
    }
}

/// Returns `true` if the given block provides strong power to the given side.
/// Note that `side` is the side of the block receiving power, not the side of the block providing power.
pub fn provides_strong_power(block: Block, side: BlockFace) -> bool {
    match block {
        Block::RedstoneTorch { .. } | Block::RedstoneWallTorch { .. } => side == BlockFace::Bottom,
        Block::TripwireHook { facing, .. } => facing.block_face() == side,
        Block::Repeater(_) | Block::Comparator(_) => provides_weak_power(block, side),
        _ if let Some((face, facing, _)) = attached::attachment(block) => {
            attached::connected_direction(face, facing) == side
        }
        _ => side == BlockFace::Top && block.pressure_plate_powered().is_some(),
    }
}

pub(crate) fn received(
    world: &impl BlockView,
    pos: BlockPos,
    direction: BlockFace,
    dust: Dust,
) -> u8 {
    let state = world.block(pos);
    let own = emitted(world, pos, state, direction, dust);
    if state.is_conductor() {
        own.max(direct_signal_to(world, pos, dust))
    } else {
        own
    }
}

pub(crate) fn has_signal(world: &impl BlockView, pos: BlockPos, direction: BlockFace) -> bool {
    received(world, pos, direction, Dust::Emits) > 0
}

pub(crate) fn has_neighbor_signal(world: &impl BlockView, pos: BlockPos) -> bool {
    DIRECTIONS
        .iter()
        .any(|&direction| has_signal(world, pos.offset(direction), direction))
}

pub(crate) fn best_neighbor_signal(world: &impl BlockView, pos: BlockPos, dust: Dust) -> u8 {
    DIRECTIONS
        .iter()
        .map(|&direction| received(world, pos.offset(direction), direction, dust))
        .max()
        .unwrap_or(0)
}

pub(crate) fn control_input(
    world: &impl BlockView,
    pos: BlockPos,
    direction: BlockFace,
    diodes_only: bool,
) -> u8 {
    let state = world.block(pos);
    match state {
        _ if diodes_only => {
            if diode::is_diode(state) {
                emitted_direct(world, pos, state, direction, Dust::Emits)
            } else {
                0
            }
        }
        Block::RedstoneBlock => 15,
        Block::RedstoneWire(wire) => wire.power,
        _ if is_signal_source(state) => emitted_direct(world, pos, state, direction, Dust::Emits),
        _ => 0,
    }
}

fn output_power(world: &impl BlockView, pos: BlockPos, state: Block) -> u8 {
    match state {
        Block::RedstoneTorch { lit } | Block::RedstoneWallTorch { lit, .. } => bool_to_ss(lit),
        Block::RedstoneBlock => 15,
        Block::TripwireHook { powered, .. } => bool_to_ss(powered),
        Block::Repeater(repeater) => bool_to_ss(repeater.powered),
        Block::Comparator(comparator) if comparator.powered => {
            comparator::output_signal(world, pos)
        }
        Block::Target { power } => power,
        _ if let Some((_, _, powered)) = attached::attachment(state) => bool_to_ss(powered),
        _ => bool_to_ss(state.pressure_plate_powered() == Some(true)),
    }
}

fn emitted(
    world: &impl BlockView,
    pos: BlockPos,
    state: Block,
    direction: BlockFace,
    dust: Dust,
) -> u8 {
    match state {
        Block::RedstoneWire(wire) if dust == Dust::Emits => {
            wire::emitted(world, pos, wire, direction)
        }
        Block::RedstoneWire(_) => 0,
        _ if provides_weak_power(state, direction) => output_power(world, pos, state),
        _ => 0,
    }
}

fn emitted_direct(
    world: &impl BlockView,
    pos: BlockPos,
    state: Block,
    direction: BlockFace,
    dust: Dust,
) -> u8 {
    match state {
        Block::RedstoneWire(_) => emitted(world, pos, state, direction, dust),
        _ if provides_strong_power(state, direction) => output_power(world, pos, state),
        _ => 0,
    }
}

fn direct_signal_to(world: &impl BlockView, pos: BlockPos, dust: Dust) -> u8 {
    DIRECTIONS
        .iter()
        .map(|&direction| {
            let neighbor_pos = pos.offset(direction);
            emitted_direct(
                world,
                neighbor_pos,
                world.block(neighbor_pos),
                direction,
                dust,
            )
        })
        .max()
        .unwrap_or(0)
}
