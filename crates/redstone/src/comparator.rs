use crate::tick_queue::TickPriority;
use crate::RedstoneWorld;
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::{Block, Comparator, ComparatorMode};
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::{BlockDirection, BlockFace, BlockPos};
use tracing::warn;

fn get_power_on_side(world: &impl BlockView, pos: BlockPos, side: BlockDirection) -> u8 {
    let side_pos = pos.offset(side.block_face());
    let side_block = world.block(side_pos);
    if super::is_diode(side_block) {
        super::get_weak_power(side_block, world, side_pos, side.block_face(), false)
    } else if let Block::RedstoneWire(wire) = side_block {
        wire.power
    } else if let Block::RedstoneBlock = side_block {
        15
    } else {
        0
    }
}

fn get_power_on_sides(comp: Comparator, world: &impl BlockView, pos: BlockPos) -> u8 {
    std::cmp::max(
        get_power_on_side(world, pos, comp.facing.rotate()),
        get_power_on_side(world, pos, comp.facing.rotate_ccw()),
    )
}

pub fn analog_output(block: Block, world: &impl BlockView, pos: BlockPos) -> Option<u8> {
    Some(match block {
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
    if !input_block.is_solid() || analog_output(input_block, world, input_pos).is_some() {
        return None;
    }

    let far_input_pos = input_pos.offset(face);
    analog_output(world.block(far_input_pos), world, far_input_pos)
}

fn calculate_input_strength(comp: Comparator, world: &impl BlockView, pos: BlockPos) -> u8 {
    let input_pos = pos.offset(comp.facing.block_face());
    let input_block = world.block(input_pos);
    if let Some(strength) = analog_output(input_block, world, input_pos) {
        return strength;
    }
    let base_input_strength = super::diode_get_input_strength(world, pos, comp.facing);
    if base_input_strength < 15 && input_block.is_solid() {
        let far_input_pos = input_pos.offset(comp.facing.block_face());
        if let Some(strength) = analog_output(world.block(far_input_pos), world, far_input_pos) {
            return strength;
        }
    }
    base_input_strength
}

pub fn should_be_powered(comp: Comparator, world: &impl BlockView, pos: BlockPos) -> bool {
    let input_strength = calculate_input_strength(comp, world, pos);
    if input_strength == 0 {
        false
    } else {
        let power_on_sides = get_power_on_sides(comp, world, pos);
        if input_strength > power_on_sides {
            true
        } else {
            power_on_sides == input_strength && comp.mode == ComparatorMode::Compare
        }
    }
}

fn calculate_output_strength(
    comp: Comparator,
    world: &mut impl RedstoneWorld,
    pos: BlockPos,
) -> u8 {
    let input_strength = calculate_input_strength(comp, world, pos);
    if comp.mode == ComparatorMode::Subtract {
        input_strength.saturating_sub(get_power_on_sides(comp, world, pos))
    } else if input_strength >= get_power_on_sides(comp, world, pos) {
        input_strength
    } else {
        0
    }
}

// This is exactly the same as it is in the RedstoneRepeater struct.
// Sometime in the future, this needs to be reused. LLVM might optimize
// it way, but te human brane wil not!
fn on_state_change(comp: Comparator, world: &mut impl RedstoneWorld, pos: BlockPos) {
    let front_pos = pos.offset(comp.facing.opposite().block_face());
    let front_block = world.block(front_pos);
    super::update(front_block, world, front_pos);
    for direction in &BlockFace::values() {
        let neighbor_pos = front_pos.offset(*direction);
        let block = world.block(neighbor_pos);
        super::update(block, world, neighbor_pos);
    }
}

fn stored_output_strength(world: &impl BlockView, pos: BlockPos) -> u8 {
    match world.block_entity(pos) {
        Some(BlockEntity::Comparator { output_strength }) => output_strength.get(),
        _ => 0,
    }
}

pub fn update(comp: Comparator, world: &mut impl RedstoneWorld, pos: BlockPos) {
    if world.pending_tick_at(pos) {
        return;
    }
    let output_strength = calculate_output_strength(comp, world, pos);
    let old_strength = stored_output_strength(world, pos);
    if output_strength != old_strength || comp.powered != should_be_powered(comp, world, pos) {
        let priority = if super::diode_should_be_prioritized(world, pos, comp.facing) {
            TickPriority::High
        } else {
            TickPriority::Normal
        };
        world.schedule_tick(pos, 1, priority);
    }
}

pub fn tick(mut comp: Comparator, world: &mut impl RedstoneWorld, pos: BlockPos) {
    let new_strength = calculate_output_strength(comp, world, pos);
    let old_strength = stored_output_strength(world, pos);
    if new_strength != old_strength || comp.mode == ComparatorMode::Compare {
        world.set_block_entity(
            pos,
            Some(BlockEntity::Comparator {
                output_strength: SignalStrength::try_from(new_strength)
                    .expect("a comparator outputs at most its input"),
            }),
        );
        let should_be_powered = should_be_powered(comp, world, pos);
        let powered = comp.powered;
        if powered && !should_be_powered {
            comp.powered = false;
            world.set_block(pos, Block::Comparator(comp));
        } else if !powered && should_be_powered {
            comp.powered = true;
            world.set_block(pos, Block::Comparator(comp));
        }
        on_state_change(comp, world, pos);
    }
}
