use crate::tick_queue::TickPriority;
use crate::update::{self, Notify};
use crate::{diode, RedstoneWorld};
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::{Block, Comparator, ComparatorMode};
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::{BlockDirection, BlockFace, BlockPos};
use tracing::warn;

pub(crate) fn output_signal(world: &impl BlockView, pos: BlockPos) -> u8 {
    match world.block_entity(pos) {
        Some(BlockEntity::Comparator { output_strength }) => output_strength.get(),
        _ => 0,
    }
}

pub fn analog_output(world: &impl BlockView, pos: BlockPos, state: Block) -> Option<u8> {
    Some(match state {
        Block::Barrel { .. } | Block::Furnace { .. } | Block::Hopper { .. } => {
            match world.block_entity(pos) {
                Some(BlockEntity::Container {
                    comparator_override,
                    ..
                }) => comparator_override.get(),
                Some(other) => {
                    warn!("Backing container blockentity type is invalid: {other:?}");
                    0
                }
                // Empty containers may not have any block entity data
                None => 0,
            }
        }
        Block::Cauldron => 0,
        Block::WaterCauldron { level } => level,
        Block::Composter { level } => level,
        Block::Cake { bites } => 14 - 2 * bites,
        Block::EndPortalFrame { eye, .. } => {
            if eye {
                15
            } else {
                0
            }
        }
        _ => return None,
    })
}

pub fn get_far_input(world: &impl BlockView, pos: BlockPos, facing: BlockDirection) -> Option<u8> {
    let face = facing.block_face();
    let input_pos = pos.offset(face);
    let input_block = world.block(input_pos);
    if !input_block.is_conductor() || analog_output(world, input_pos, input_block).is_some() {
        return None;
    }

    let far_input_pos = input_pos.offset(face);
    analog_output(world, far_input_pos, world.block(far_input_pos))
}

pub(crate) fn neighbor_changed(
    world: &mut impl RedstoneWorld,
    pos: BlockPos,
    comparator: Comparator,
) {
    if !diode::can_survive(world, pos) {
        diode::remove_unsupported(world, pos);
        return;
    }
    let state = Block::Comparator(comparator);
    if world.redstone().ticks.is_pending(pos, state) {
        return;
    }
    let output = calculate_output(world, pos, comparator);
    if output != output_signal(world, pos)
        || comparator.powered != should_turn_on(world, pos, comparator)
    {
        let priority = if diode::diode_should_be_prioritized(world, pos, comparator.facing) {
            TickPriority::High
        } else {
            TickPriority::Normal
        };
        world.schedule_tick(pos, 1, priority, state);
    }
}

pub(crate) fn tick(world: &mut impl RedstoneWorld, pos: BlockPos, comparator: Comparator) {
    let output = calculate_output(world, pos, comparator);
    let old_output = output_signal(world, pos);
    if old_output != output {
        world.set_block_entity(
            pos,
            Some(BlockEntity::Comparator {
                output_strength: SignalStrength::try_from(output)
                    .expect("a comparator outputs at most its input"),
            }),
        );
    } else if comparator.mode != ComparatorMode::Compare {
        return;
    }
    let should_be_powered = should_turn_on(world, pos, comparator);
    if comparator.powered != should_be_powered {
        let toggled = Comparator {
            powered: should_be_powered,
            ..comparator
        };
        update::set_block(world, pos, Block::Comparator(toggled), Notify::Shapes);
    }
    diode::update_neighbors_in_front(world, pos, comparator.facing.block_face());
}

pub(crate) fn update_shape(
    comparator: Comparator,
    direction: BlockFace,
    neighbor_state: Block,
) -> Block {
    if direction == BlockFace::Bottom && !neighbor_state.can_support_rigid_block() {
        Block::Air
    } else {
        Block::Comparator(comparator)
    }
}

pub(crate) fn use_block(world: &mut impl RedstoneWorld, pos: BlockPos, comparator: Comparator) {
    let toggled = Comparator {
        mode: comparator.mode.toggle(),
        ..comparator
    };
    update::set_block(world, pos, Block::Comparator(toggled), Notify::Shapes);
    tick(world, pos, toggled);
}

fn calculate_output(world: &impl BlockView, pos: BlockPos, comparator: Comparator) -> u8 {
    let input = input_signal(world, pos, comparator.facing);
    let side = diode::side_signal(world, pos, comparator.facing, false);
    match comparator.mode {
        _ if input == 0 || side > input => 0,
        ComparatorMode::Subtract => input - side,
        ComparatorMode::Compare => input,
    }
}

fn should_turn_on(world: &impl BlockView, pos: BlockPos, comparator: Comparator) -> bool {
    let input = input_signal(world, pos, comparator.facing);
    let side = diode::side_signal(world, pos, comparator.facing, false);
    input > 0 && (input > side || (input == side && comparator.mode == ComparatorMode::Compare))
}

fn input_signal(world: &impl BlockView, pos: BlockPos, facing: BlockDirection) -> u8 {
    let signal = diode::input_signal(world, pos, facing);
    let input_pos = pos.offset(facing.block_face());
    let input = world.block(input_pos);
    if let Some(analog) = analog_output(world, input_pos, input) {
        analog
    } else if signal < 15 && input.is_conductor() {
        get_far_input(world, pos, facing).unwrap_or(signal)
    } else {
        signal
    }
}
