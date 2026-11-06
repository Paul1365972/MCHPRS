use crate::update::{self, Notify};
use crate::RedstoneWorld;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::{BlockFace, BlockPos};

pub(crate) fn can_survive(world: &impl BlockView, pos: BlockPos) -> bool {
    let below = world.block(pos.offset(BlockFace::Bottom));
    below.can_support_rigid_block() || below.can_support_center()
}

pub(crate) fn update_neighbors(world: &mut impl RedstoneWorld, pos: BlockPos) {
    update::update_neighbors_at(world, pos);
    update::update_neighbors_at(world, pos.offset(BlockFace::Bottom));
}

pub fn set_pressure_plate(world: &mut impl RedstoneWorld, pos: BlockPos, pressed: bool) {
    let state = with_powered(world.block(pos), pressed);
    if update::set_block(world, pos, state, Notify::Shapes) {
        update_neighbors(world, pos);
    }
}

fn with_powered(mut state: Block, powered: bool) -> Block {
    if let Some(plate_powered) = state.get_pressure_plate_powered() {
        *plate_powered = powered;
    }
    state
}
