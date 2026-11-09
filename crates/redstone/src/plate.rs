use crate::tick_queue::TickPriority;
use crate::update::{self, Notify};
use crate::RedstoneWorld;
pub const PRESS_TICKS: u32 = 10;
pub const RECHECK_PRIORITY: TickPriority = TickPriority::Normal;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::{BlockFace, BlockPos};

pub(crate) fn can_survive_on(below: Block) -> bool {
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
    if let Some(plate) = state.pressure_plate_mut() {
        plate.powered = powered;
    }
    state
}
