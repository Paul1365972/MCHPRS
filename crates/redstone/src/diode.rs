use crate::signal::{self, Dust};
use crate::update::{self, Notify};
use crate::{RedstoneWorld, DIRECTIONS};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::{BlockDirection, BlockFace, BlockPos};

pub fn is_diode(block: Block) -> bool {
    matches!(block, Block::Repeater(_) | Block::Comparator(_))
}

pub(crate) fn can_survive(world: &impl BlockView, pos: BlockPos) -> bool {
    world
        .block(pos.offset(BlockFace::Bottom))
        .can_support_rigid_block()
}

pub(crate) fn remove_unsupported(world: &mut impl RedstoneWorld, pos: BlockPos) {
    update::set_block(world, pos, Block::Air, Notify::All);
    for direction in DIRECTIONS {
        update::update_neighbors_at(world, pos.offset(direction));
    }
}

pub(crate) fn input_signal(world: &impl BlockView, pos: BlockPos, facing: BlockDirection) -> u8 {
    let input_pos = pos.offset(facing.block_face());
    let signal = signal::received(world, input_pos, facing.block_face(), Dust::Emits);
    if signal >= 15 {
        return signal;
    }
    match world.block(input_pos) {
        Block::RedstoneWire(wire) => signal.max(wire.power),
        _ => signal,
    }
}

pub(crate) fn side_signal(
    world: &impl BlockView,
    pos: BlockPos,
    facing: BlockDirection,
    diodes_only: bool,
) -> u8 {
    [facing.rotate(), facing.rotate_ccw()]
        .iter()
        .map(|side| {
            let face = side.block_face();
            signal::control_input(world, pos.offset(face), face, diodes_only)
        })
        .max()
        .unwrap_or(0)
}

pub fn diode_should_be_prioritized(
    world: &impl BlockView,
    pos: BlockPos,
    facing: BlockDirection,
) -> bool {
    let output_direction = facing.opposite();
    match world.block(pos.offset(output_direction.block_face())) {
        Block::Repeater(front_repeater) => front_repeater.facing != output_direction,
        Block::Comparator(front_comparator) => front_comparator.facing != output_direction,
        _ => false,
    }
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
