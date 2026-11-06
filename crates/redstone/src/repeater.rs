use crate::tick_queue::TickPriority;
use crate::update::{self, Notify};
use crate::{diode, RedstoneWorld};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::{Block, Repeater};
use mchprs_blocks::{BlockDirection, BlockFace, BlockPos};

pub(crate) fn state_for_placement(
    world: &impl BlockView,
    pos: BlockPos,
    repeater: Repeater,
) -> Repeater {
    Repeater {
        locked: is_locked(world, pos, repeater.facing),
        ..repeater
    }
}

pub(crate) fn neighbor_changed(world: &mut impl RedstoneWorld, pos: BlockPos, repeater: Repeater) {
    if !diode::can_survive(world, pos) {
        diode::remove_unsupported(world, pos);
        return;
    }
    if is_locked(world, pos, repeater.facing) {
        return;
    }
    let should_be_powered = should_turn_on(world, pos, repeater);
    let state = Block::Repeater(repeater);
    if repeater.powered != should_be_powered && !world.redstone().ticks.is_pending(pos, state) {
        let priority = if diode::diode_should_be_prioritized(world, pos, repeater.facing) {
            TickPriority::Highest
        } else if repeater.powered {
            TickPriority::Higher
        } else {
            TickPriority::High
        };
        world.schedule_tick(pos, repeater.delay as u32, priority, state);
    }
}

pub(crate) fn tick(world: &mut impl RedstoneWorld, pos: BlockPos, repeater: Repeater) {
    if is_locked(world, pos, repeater.facing) {
        return;
    }
    let should_be_powered = should_turn_on(world, pos, repeater);
    if repeater.powered && !should_be_powered {
        let unpowered = Repeater {
            powered: false,
            ..repeater
        };
        update::set_block(world, pos, Block::Repeater(unpowered), Notify::Shapes);
    } else if !repeater.powered {
        let powered = Repeater {
            powered: true,
            ..repeater
        };
        update::set_block(world, pos, Block::Repeater(powered), Notify::Shapes);
        if !should_be_powered {
            world.schedule_tick(
                pos,
                repeater.delay as u32,
                TickPriority::Higher,
                Block::Repeater(repeater),
            );
        }
    }
}

pub(crate) fn update_shape(
    world: &impl BlockView,
    pos: BlockPos,
    repeater: Repeater,
    direction: BlockFace,
    neighbor_state: Block,
) -> Block {
    if direction == BlockFace::Bottom && !neighbor_state.can_support_rigid_block() {
        return Block::Air;
    }
    let along_axis = direction == repeater.facing.block_face()
        || direction == repeater.facing.opposite().block_face();
    if along_axis {
        return Block::Repeater(repeater);
    }
    Block::Repeater(Repeater {
        locked: is_locked(world, pos, repeater.facing),
        ..repeater
    })
}

pub(crate) fn use_block(world: &mut impl RedstoneWorld, pos: BlockPos, repeater: Repeater) {
    let cycled = Repeater {
        delay: repeater.delay % 4 + 1,
        ..repeater
    };
    update::set_block(world, pos, Block::Repeater(cycled), Notify::All);
}

fn is_locked(world: &impl BlockView, pos: BlockPos, facing: BlockDirection) -> bool {
    diode::side_signal(world, pos, facing, true) > 0
}

fn should_turn_on(world: &impl BlockView, pos: BlockPos, repeater: Repeater) -> bool {
    diode::input_signal(world, pos, repeater.facing) > 0
}
