use crate::{attached, comparator, diode, wire, DIRECTIONS};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::signal_strength::{SignalStrength, UncappedSignalStrength};
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
) -> UncappedSignalStrength {
    let state = world.block(pos);
    let own = emitted(world, pos, state, direction, dust);
    if state.is_conductor() {
        own.max(direct_signal_to(world, pos, dust))
    } else {
        own
    }
}

pub(crate) fn has_signal(world: &impl BlockView, pos: BlockPos, direction: BlockFace) -> bool {
    !received(world, pos, direction, Dust::Emits).is_zero()
}

pub(crate) fn has_neighbor_signal(world: &impl BlockView, pos: BlockPos) -> bool {
    DIRECTIONS
        .iter()
        .any(|&direction| has_signal(world, pos.offset(direction), direction))
}

pub(crate) fn best_neighbor_signal(
    world: &impl BlockView,
    pos: BlockPos,
    dust: Dust,
) -> SignalStrength {
    DIRECTIONS
        .iter()
        .map(|&direction| received(world, pos.offset(direction), direction, dust))
        .max()
        .unwrap_or(UncappedSignalStrength::ZERO)
        .capped()
}

pub(crate) fn control_input(
    world: &impl BlockView,
    pos: BlockPos,
    direction: BlockFace,
    diodes_only: bool,
) -> UncappedSignalStrength {
    let state = world.block(pos);
    match state {
        _ if diodes_only => {
            if diode::is_diode(state) {
                emitted_direct(world, pos, state, direction, Dust::Emits)
            } else {
                UncappedSignalStrength::ZERO
            }
        }
        Block::RedstoneBlock => SignalStrength::Fifteen.into(),
        Block::RedstoneWire(wire) => wire.power.into(),
        _ if is_signal_source(state) => emitted_direct(world, pos, state, direction, Dust::Emits),
        _ => UncappedSignalStrength::ZERO,
    }
}

fn output_power(world: &impl BlockView, pos: BlockPos, state: Block) -> UncappedSignalStrength {
    match state {
        Block::RedstoneTorch { lit } | Block::RedstoneWallTorch { lit, .. } => {
            SignalStrength::from(lit).into()
        }
        Block::RedstoneBlock => SignalStrength::Fifteen.into(),
        Block::TripwireHook { powered, .. } => SignalStrength::from(powered).into(),
        Block::Repeater(repeater) => SignalStrength::from(repeater.powered).into(),
        Block::Comparator(comparator) if comparator.powered => {
            comparator::output_signal(world, pos)
        }
        Block::Target { power } => power.into(),
        _ if let Some((_, _, powered)) = attached::attachment(state) => {
            SignalStrength::from(powered).into()
        }
        _ => SignalStrength::from(state.pressure_plate_powered() == Some(true)).into(),
    }
}

fn emitted(
    world: &impl BlockView,
    pos: BlockPos,
    state: Block,
    direction: BlockFace,
    dust: Dust,
) -> UncappedSignalStrength {
    match state {
        Block::RedstoneWire(wire) if dust == Dust::Emits => {
            wire::emitted(world, pos, wire, direction).into()
        }
        Block::RedstoneWire(_) => UncappedSignalStrength::ZERO,
        _ if provides_weak_power(state, direction) => output_power(world, pos, state),
        _ => UncappedSignalStrength::ZERO,
    }
}

fn emitted_direct(
    world: &impl BlockView,
    pos: BlockPos,
    state: Block,
    direction: BlockFace,
    dust: Dust,
) -> UncappedSignalStrength {
    match state {
        Block::RedstoneWire(_) => emitted(world, pos, state, direction, dust),
        _ if provides_strong_power(state, direction) => output_power(world, pos, state),
        _ => UncappedSignalStrength::ZERO,
    }
}

fn direct_signal_to(world: &impl BlockView, pos: BlockPos, dust: Dust) -> UncappedSignalStrength {
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
        .unwrap_or(UncappedSignalStrength::ZERO)
}
