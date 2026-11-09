use crate::tick_queue::TickPriority;
use crate::update::{self, Notify};
use crate::{Aabb, RedstoneWorld};
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::{BlockFace, BlockPos};

const TOUCH_AREA: Aabb = Aabb {
    min: [0.0625, 0.0, 0.0625],
    max: [0.9375, 0.25, 0.9375],
};

pub const PRESS_TICKS: u32 = 10;
pub const RECHECK_PRIORITY: TickPriority = TickPriority::Normal;

pub(crate) fn can_survive_on(below: Block) -> bool {
    below.can_support_rigid_block() || below.can_support_center()
}

pub(crate) fn update_neighbors(world: &mut impl RedstoneWorld, pos: BlockPos) {
    update::update_neighbors_at(world, pos);
    update::update_neighbors_at(world, pos.offset(BlockFace::Bottom));
}

pub fn entity_inside(world: &mut impl RedstoneWorld, pos: BlockPos) -> bool {
    let state = world.block(pos);
    state.pressure_plate().is_some_and(|plate| !plate.powered) && check_pressed(world, pos, state)
}

pub(crate) fn tick(world: &mut impl RedstoneWorld, pos: BlockPos, state: Block) {
    if state.pressure_plate().is_some_and(|plate| plate.powered) {
        check_pressed(world, pos, state);
    }
}

pub fn is_touched(world: &impl RedstoneWorld, pos: BlockPos) -> bool {
    world.has_entity_in(TOUCH_AREA.offset(pos))
}

pub(crate) fn schedule_recheck(world: &mut impl RedstoneWorld, pos: BlockPos, state: Block) {
    world.schedule_tick(pos, PRESS_TICKS, RECHECK_PRIORITY, state);
}

pub fn recheck_pressed_plates(world: &mut impl RedstoneWorld, region: Region) {
    for pos in world.occupied_positions(region) {
        let block = world.block(pos);
        if block.pressure_plate().is_some_and(|plate| plate.powered) {
            schedule_recheck(world, pos, block);
        }
    }
}

fn check_pressed(world: &mut impl RedstoneWorld, pos: BlockPos, state: Block) -> bool {
    let pressed = is_touched(world, pos);
    if state.pressure_plate().map(|plate| plate.powered) != Some(pressed) {
        update::set_block(world, pos, with_powered(state, pressed), Notify::Shapes);
        update_neighbors(world, pos);
    }
    if pressed {
        schedule_recheck(world, pos, state);
    }
    pressed
}

fn with_powered(mut state: Block, powered: bool) -> Block {
    if let Some(plate) = state.pressure_plate_mut() {
        plate.powered = powered;
    }
    state
}
