use crate::signal::{self, Dust};
use crate::update::{self, Notify};
use crate::RedstoneWorld;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::{Block, Comparator, Repeater};
use mchprs_blocks::signal_strength::{SignalStrength, UncappedSignalStrength};
use mchprs_blocks::{BlockDirection, BlockFace, BlockPos};

pub fn is_diode(block: Block) -> bool {
    matches!(block, Block::Repeater(_) | Block::Comparator(_))
}

pub(crate) fn facing(block: Block) -> Option<BlockDirection> {
    match block {
        Block::Repeater(Repeater { facing, .. }) | Block::Comparator(Comparator { facing, .. }) => {
            Some(facing)
        }
        _ => None,
    }
}

pub(crate) fn can_survive(world: &impl BlockView, pos: BlockPos) -> bool {
    world
        .block(pos.offset(BlockFace::Bottom))
        .can_support_rigid_block()
}

pub(crate) fn remove_unsupported(world: &mut impl RedstoneWorld, pos: BlockPos) {
    update::set_block(world, pos, Block::Air, Notify::All);
    update::update_neighbors_around(world, pos);
}

pub(crate) fn input_signal(
    world: &impl BlockView,
    pos: BlockPos,
    facing: BlockDirection,
) -> UncappedSignalStrength {
    let input_pos = pos.offset(facing.block_face());
    let signal = signal::received(world, input_pos, facing.block_face(), Dust::Emits);
    if signal >= SignalStrength::Fifteen.into() {
        return signal;
    }
    match world.block(input_pos) {
        Block::RedstoneWire(wire) => signal.max(wire.power.into()),
        _ => signal,
    }
}

pub(crate) fn side_signal(
    world: &impl BlockView,
    pos: BlockPos,
    facing: BlockDirection,
) -> UncappedSignalStrength {
    [facing.rotate(), facing.rotate_ccw()]
        .iter()
        .map(|side| {
            let face = side.block_face();
            signal::control_input(world, pos.offset(face), face)
        })
        .max()
        .unwrap_or(UncappedSignalStrength::ZERO)
}

pub fn is_prioritized(world: &impl BlockView, pos: BlockPos, own_facing: BlockDirection) -> bool {
    let output_direction = own_facing.opposite();
    let front = world.block(pos.offset(output_direction.block_face()));
    facing(front).is_some_and(|front_facing| front_facing != output_direction)
}

pub(crate) fn update_neighbors_in_front(
    world: &mut impl RedstoneWorld,
    pos: BlockPos,
    facing: BlockFace,
) {
    let front_pos = pos.offset(facing.opposite());
    update::neighbor_changed_at(world, front_pos);
    update::update_neighbors_at_except(world, front_pos, Some(facing));
}
