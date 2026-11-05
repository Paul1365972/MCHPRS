use crate::extract::ExtractError;
use crate::graph::{NodeKind, NodeState};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::signal_strength::{SignalStrength, UncappedSignalStrength};
use mchprs_blocks::BlockPos;
use mchprs_redstone::{attached, comparator, diode, noteblock};
use mchprs_simulation::component::strength_of;

pub fn component_at(
    world: &impl BlockView,
    pos: BlockPos,
) -> Result<Option<(NodeKind, NodeState)>, ExtractError> {
    component_of(world, pos, world.block(pos))
}

pub(crate) fn component_of(
    world: &impl BlockView,
    pos: BlockPos,
    block: Block,
) -> Result<Option<(NodeKind, NodeState)>, ExtractError> {
    let Some(strength) = strength_of(world, pos, block) else {
        return Ok(None);
    };
    let power = within_fifteen(strength, pos)?;
    let kind = match block {
        _ if block.pressure_plate().is_some() => NodeKind::PressurePlate,
        Block::Repeater(repeater) => NodeKind::Repeater {
            delay: repeater.delay,
            prioritized: diode::is_prioritized(world, pos, repeater.facing),
        },
        Block::Comparator(comparator) => NodeKind::Comparator {
            mode: comparator.mode,
            far_input: comparator::far_input(world, pos, comparator.facing)
                .map(|power| within_fifteen(power, pos))
                .transpose()?,
            prioritized: diode::is_prioritized(world, pos, comparator.facing),
        },
        Block::RedstoneTorch { .. } | Block::RedstoneWallTorch { .. } => NodeKind::Torch,
        Block::RedstoneWire(_) => NodeKind::Wire,
        _ if block.button().is_some() => NodeKind::Button {
            press_ticks: attached::button_press_ticks(block),
        },
        Block::RedstoneLamp { .. } => NodeKind::Lamp,
        Block::Lever { .. } => NodeKind::Lever,
        Block::IronTrapdoor { .. } => NodeKind::Trapdoor,
        Block::NoteBlock {
            instrument, note, ..
        } if noteblock::is_unblocked(world, pos) => NodeKind::NoteBlock { instrument, note },
        Block::NoteBlock { .. } => NodeKind::Trapdoor,
        _ => NodeKind::Constant,
    };
    let state = match block {
        Block::Repeater(repeater) => NodeState::repeater(repeater.powered, repeater.locked),
        _ => NodeState::from_power(power),
    };
    Ok(Some((kind, state)))
}

fn within_fifteen(
    strength: UncappedSignalStrength,
    pos: BlockPos,
) -> Result<SignalStrength, ExtractError> {
    SignalStrength::try_from(strength).map_err(|_| ExtractError::AboveFifteen(pos))
}

pub fn apply_state(block: &mut Block, state: NodeState) {
    if let Some(powered) = powered_mut(block) {
        *powered = state.is_powered();
    }
    match block {
        Block::IronTrapdoor { open, .. } => *open = state.is_powered(),
        Block::RedstoneWire(wire) => wire.power = state.power,
        Block::Repeater(repeater) => repeater.locked = state.repeater_locked,
        _ => {}
    }
}

fn powered_mut(block: &mut Block) -> Option<&mut bool> {
    Some(match block {
        Block::Comparator(comparator) => &mut comparator.powered,
        Block::RedstoneTorch { lit } => lit,
        Block::RedstoneWallTorch { lit, .. } => lit,
        Block::Repeater(repeater) => &mut repeater.powered,
        Block::Lever { powered, .. } => powered,
        Block::RedstoneLamp { lit } => lit,
        Block::IronTrapdoor { powered, .. } => powered,
        Block::NoteBlock { powered, .. } => powered,
        _ if block.button().is_some() => &mut block.button_mut()?.powered,
        _ => return block.pressure_plate_mut().map(|plate| &mut plate.powered),
    })
}
