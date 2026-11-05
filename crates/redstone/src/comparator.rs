use crate::tick_queue::TickPriority;
use crate::update::{self, Notify};
use crate::{diode, RedstoneWorld};
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::{Block, Comparator, ComparatorMode};
use mchprs_blocks::signal_strength::{SignalStrength, UncappedSignalStrength};
use mchprs_blocks::{BlockDirection, BlockFace, BlockPos};
use tracing::warn;

pub const DELAY_TICKS: u32 = 1;

pub fn output_signal(world: &impl BlockView, pos: BlockPos) -> UncappedSignalStrength {
    match world.block_entity(pos) {
        Some(BlockEntity::Comparator { output_strength }) => *output_strength,
        _ => UncappedSignalStrength::ZERO,
    }
}

pub fn tick_priority(prioritized: bool) -> TickPriority {
    if prioritized {
        TickPriority::High
    } else {
        TickPriority::Normal
    }
}

pub fn analog_output(
    world: &impl BlockView,
    pos: BlockPos,
    state: Block,
) -> Option<UncappedSignalStrength> {
    Some(match state {
        Block::Barrel { .. } | Block::Furnace { .. } | Block::Hopper { .. } => {
            match world.block_entity(pos) {
                Some(BlockEntity::Container { inventory, ty }) => ty.signal_strength(inventory),
                Some(other) => {
                    warn!("Backing container blockentity type is invalid: {other:?}");
                    UncappedSignalStrength::ZERO
                }
                None => UncappedSignalStrength::ZERO,
            }
        }
        Block::Cauldron => UncappedSignalStrength::ZERO,
        Block::WaterCauldron { level } => UncappedSignalStrength::new(level.into()),
        Block::Composter { level } => UncappedSignalStrength::new(level.into()),
        Block::Cake { bites } => UncappedSignalStrength::new((14 - 2 * bites).into()),
        Block::EndPortalFrame { eye, .. } => SignalStrength::from(eye).into(),
        _ => return None,
    })
}

pub fn far_input(
    world: &impl BlockView,
    pos: BlockPos,
    facing: BlockDirection,
) -> Option<UncappedSignalStrength> {
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
        let prioritized = diode::is_prioritized(world, pos, comparator.facing);
        world.schedule_tick(pos, DELAY_TICKS, tick_priority(prioritized), state);
    }
}

pub(crate) fn tick(world: &mut impl RedstoneWorld, pos: BlockPos, comparator: Comparator) {
    let output = calculate_output(world, pos, comparator);
    let old_output = output_signal(world, pos);
    if old_output != output {
        world.set_block_entity(
            pos,
            Some(BlockEntity::Comparator {
                output_strength: output,
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

fn calculate_output(
    world: &impl BlockView,
    pos: BlockPos,
    comparator: Comparator,
) -> UncappedSignalStrength {
    let input = input_signal(world, pos, comparator.facing);
    let side = diode::side_signal(world, pos, comparator.facing);
    comparator.mode.output(input, side)
}

fn should_turn_on(world: &impl BlockView, pos: BlockPos, comparator: Comparator) -> bool {
    let input = input_signal(world, pos, comparator.facing);
    let side = diode::side_signal(world, pos, comparator.facing);
    !input.is_zero()
        && (input > side || (input == side && comparator.mode == ComparatorMode::Compare))
}

fn input_signal(
    world: &impl BlockView,
    pos: BlockPos,
    facing: BlockDirection,
) -> UncappedSignalStrength {
    let signal = diode::input_signal(world, pos, facing);
    let input_pos = pos.offset(facing.block_face());
    let input = world.block(input_pos);
    if let Some(analog) = analog_output(world, input_pos, input) {
        analog
    } else if signal < SignalStrength::Fifteen.into() && input.is_conductor() {
        far_input(world, pos, facing).unwrap_or(signal)
    } else {
        signal
    }
}
