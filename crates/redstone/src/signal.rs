use crate::{attached, comparator, diode, wire, DIRECTIONS};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::signal_strength::{SignalStrength, UncappedSignalStrength};
use mchprs_blocks::{BlockFace, BlockPos};

/// Vanilla's `RedStoneWireBlock.shouldSignal`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dust {
    Emits,
    Muted,
}

pub fn is_signal_source(state: Block) -> bool {
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
        || state.pressure_plate().is_some()
}

/// `direction` points from the neighbor that reads `block` to `block`.
pub fn provides_weak_power(block: Block, direction: BlockFace) -> bool {
    match block {
        Block::RedstoneTorch { .. } => direction != BlockFace::Top,
        Block::RedstoneWallTorch { facing, .. } => facing.block_face() != direction,
        Block::RedstoneBlock | Block::TripwireHook { .. } | Block::Target { .. } => true,
        _ if let Some(facing) = diode::facing(block) => facing.block_face() == direction,
        _ => attached::attachment(block).is_some() || block.pressure_plate().is_some(),
    }
}

pub fn provides_strong_power(block: Block, direction: BlockFace) -> bool {
    match block {
        Block::RedstoneTorch { .. } | Block::RedstoneWallTorch { .. } => {
            direction == BlockFace::Bottom
        }
        Block::TripwireHook { facing, .. } => facing.block_face() == direction,
        Block::Repeater(_) | Block::Comparator(_) => provides_weak_power(block, direction),
        _ if let Some((face, facing, _)) = attached::attachment(block) => {
            attached::connected_direction(face, facing) == direction
        }
        _ => direction == BlockFace::Top && block.pressure_plate().is_some(),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Emitter {
    pub pos: BlockPos,
    pub state: Block,
    pub through_conductor: bool,
}

pub fn emitters(
    world: &impl BlockView,
    pos: BlockPos,
    direction: BlockFace,
    dust: Dust,
) -> impl Iterator<Item = Emitter> + '_ {
    let state = world.block(pos);
    let own = emits(world, pos, state, direction, false, dust).then_some(Emitter {
        pos,
        state,
        through_conductor: false,
    });
    let conductor = state.is_conductor();
    // Link search calls this for every wire, and flattening an optional array here is slower.
    let conducted = DIRECTIONS
        .into_iter()
        .filter(move |_| conductor)
        .filter_map(move |direction| {
            let neighbor_pos = pos.offset(direction);
            let neighbor = world.block(neighbor_pos);
            emits(world, neighbor_pos, neighbor, direction, true, dust).then_some(Emitter {
                pos: neighbor_pos,
                state: neighbor,
                through_conductor: true,
            })
        });
    own.into_iter().chain(conducted)
}

fn emits(
    world: &impl BlockView,
    pos: BlockPos,
    state: Block,
    direction: BlockFace,
    strongly: bool,
    dust: Dust,
) -> bool {
    match state {
        Block::RedstoneWire(wire) => {
            dust == Dust::Emits && wire::emits(world, pos, wire, direction)
        }
        _ if strongly => provides_strong_power(state, direction),
        _ => provides_weak_power(state, direction),
    }
}

pub fn received(
    world: &impl BlockView,
    pos: BlockPos,
    direction: BlockFace,
    dust: Dust,
) -> UncappedSignalStrength {
    let mut own = UncappedSignalStrength::ZERO;
    let mut conducted = UncappedSignalStrength::ZERO;
    for emitter in emitters(world, pos, direction, dust) {
        let strength = match emitter.state {
            Block::RedstoneWire(wire) => wire.power.into(),
            state => output_power(world, emitter.pos, state),
        };
        if !emitter.through_conductor {
            own = strength;
            continue;
        }
        conducted = conducted.max(strength);
        if conducted >= SignalStrength::Fifteen.into() {
            break;
        }
    }
    own.max(conducted)
}

pub fn has_signal(world: &impl BlockView, pos: BlockPos, direction: BlockFace) -> bool {
    !received(world, pos, direction, Dust::Emits).is_zero()
}

pub fn has_neighbor_signal(world: &impl BlockView, pos: BlockPos) -> bool {
    DIRECTIONS
        .iter()
        .any(|&direction| has_signal(world, pos.offset(direction), direction))
}

pub fn best_neighbor_signal(world: &impl BlockView, pos: BlockPos, dust: Dust) -> SignalStrength {
    DIRECTIONS
        .iter()
        .map(|&direction| received(world, pos.offset(direction), direction, dust))
        .max()
        .unwrap_or(UncappedSignalStrength::ZERO)
        .capped()
}

pub fn control_input(
    world: &impl BlockView,
    pos: BlockPos,
    direction: BlockFace,
) -> UncappedSignalStrength {
    let state = world.block(pos);
    match state {
        Block::RedstoneBlock => SignalStrength::Fifteen.into(),
        Block::RedstoneWire(wire) => wire.power.into(),
        _ if is_signal_source(state) => emitted_direct(world, pos, state, direction),
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
        _ => SignalStrength::from(state.pressure_plate().is_some_and(|plate| plate.powered)).into(),
    }
}

pub fn emitted_direct(
    world: &impl BlockView,
    pos: BlockPos,
    state: Block,
    direction: BlockFace,
) -> UncappedSignalStrength {
    if provides_strong_power(state, direction) {
        output_power(world, pos, state)
    } else {
        UncappedSignalStrength::ZERO
    }
}
