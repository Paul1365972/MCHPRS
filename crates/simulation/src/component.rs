use crate::ports::Role;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::BlockPos;
use mchprs_redstone::comparator;
use mchprs_redstone::input::is_drivable;

pub fn strength_at(world: &impl BlockView, pos: BlockPos) -> Option<u8> {
    strength_of(world, pos, world.block(pos))
}

pub fn strength_of(world: &impl BlockView, pos: BlockPos, block: Block) -> Option<u8> {
    if let Some(on) = switched_on(block) {
        return Some(SignalStrength::from(on).into());
    }
    match block {
        Block::RedstoneWire(wire) => Some(wire.power.into()),
        Block::Comparator(_) => Some(comparator::output_signal(world, pos)),
        Block::RedstoneBlock | Block::TripwireHook { powered: true, .. } => {
            Some(SignalStrength::Fifteen.into())
        }
        Block::Target { power } if !power.is_zero() => Some(power.into()),
        _ => comparator::analog_output(world, pos, block),
    }
}

pub fn powered_at(world: &impl BlockView, pos: BlockPos) -> Option<bool> {
    strength_at(world, pos).map(|strength| strength != 0)
}

pub fn default_role(block: Block) -> Option<Role> {
    match block {
        _ if is_drivable(block) => Some(Role::Input),
        Block::RedstoneLamp { .. } | Block::IronTrapdoor { .. } | Block::NoteBlock { .. } => {
            Some(Role::Output)
        }
        _ => None,
    }
}

fn switched_on(block: Block) -> Option<bool> {
    Some(match block {
        Block::Repeater(repeater) => repeater.powered,
        Block::RedstoneTorch { lit } | Block::RedstoneWallTorch { lit, .. } => lit,
        Block::RedstoneLamp { lit } => lit,
        Block::Lever { powered, .. }
        | Block::IronTrapdoor { powered, .. }
        | Block::NoteBlock { powered, .. } => powered,
        _ if let Some(button) = block.button() => button.powered,
        _ => block.pressure_plate()?.powered,
    })
}
