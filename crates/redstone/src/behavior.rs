use crate::tick_queue::TickPriority;
use crate::update::{self, Notify};
use crate::{
    attached, comparator, diode, noteblock, plate, repeater, signal, torch, wire, RedstoneWorld,
};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::{BlockFace, BlockPos};

const LAMP_OFF_DELAY: u32 = 2;

pub(crate) fn neighbor_changed(world: &mut impl RedstoneWorld, pos: BlockPos, state: Block) {
    match state {
        Block::RedstoneWire(wire) => wire::neighbor_changed(world, pos, wire),
        Block::Repeater(repeater) => repeater::neighbor_changed(world, pos, repeater),
        Block::Comparator(comparator) => comparator::neighbor_changed(world, pos, comparator),
        Block::RedstoneTorch { .. } | Block::RedstoneWallTorch { .. } => {
            torch::neighbor_changed(world, pos, state)
        }
        Block::RedstoneLamp { lit } => {
            if lit == signal::has_neighbor_signal(world, pos) {
                return;
            }
            if lit {
                world.schedule_tick(pos, LAMP_OFF_DELAY, TickPriority::Normal, state);
            } else {
                update::set_block(
                    world,
                    pos,
                    Block::RedstoneLamp { lit: true },
                    Notify::Shapes,
                );
            }
        }
        Block::IronTrapdoor {
            facing,
            half,
            powered,
            waterlogged,
            ..
        } => {
            let should_be_powered = signal::has_neighbor_signal(world, pos);
            if should_be_powered != powered {
                let toggled = Block::IronTrapdoor {
                    facing,
                    half,
                    open: should_be_powered,
                    powered: should_be_powered,
                    waterlogged,
                };
                update::set_block(world, pos, toggled, Notify::Shapes);
            }
        }
        Block::NoteBlock {
            instrument,
            note,
            powered,
        } => noteblock::neighbor_changed(world, pos, instrument, note, powered),
        Block::Hopper { .. } => update_hopper(world, pos, state),
        _ => {}
    }
}

pub(crate) fn tick(world: &mut impl RedstoneWorld, pos: BlockPos, state: Block) {
    match state {
        Block::Repeater(repeater) => repeater::tick(world, pos, repeater),
        Block::Comparator(comparator) => comparator::tick(world, pos, comparator),
        Block::RedstoneTorch { .. } | Block::RedstoneWallTorch { .. } => {
            torch::tick(world, pos, state)
        }
        Block::RedstoneLamp { lit: true } if !signal::has_neighbor_signal(world, pos) => {
            update::set_block(
                world,
                pos,
                Block::RedstoneLamp { lit: false },
                Notify::Shapes,
            );
        }
        _ if state.button().is_some_and(|button| button.powered) => {
            attached::set_button_powered(world, pos, state, false)
        }
        _ => {}
    }
}

pub(crate) fn update_shape(
    world: &impl BlockView,
    pos: BlockPos,
    state: Block,
    direction: BlockFace,
    neighbor_state: Block,
) -> Block {
    match state {
        Block::RedstoneWire(wire) => {
            wire::update_shape(world, pos, wire, direction, neighbor_state)
        }
        Block::Repeater(repeater) => {
            repeater::update_shape(world, pos, repeater, direction, neighbor_state)
        }
        Block::Comparator(comparator) => {
            comparator::update_shape(comparator, direction, neighbor_state)
        }
        Block::NoteBlock { .. } => noteblock::update_shape(world, pos, state, direction),
        _ if support_face(state) == Some(direction) && !can_survive(world, pos, state) => {
            Block::Air
        }
        _ => state,
    }
}

pub(crate) fn on_place(world: &mut impl RedstoneWorld, pos: BlockPos, state: Block, old: Block) {
    match state {
        Block::RedstoneWire(wire) => wire::on_place(world, pos, wire, old),
        Block::Repeater(repeater) => {
            diode::update_neighbors_in_front(world, pos, repeater.facing.block_face())
        }
        Block::Comparator(comparator) => {
            diode::update_neighbors_in_front(world, pos, comparator.facing.block_face())
        }
        Block::RedstoneTorch { .. } | Block::RedstoneWallTorch { .. } => {
            torch::update_surroundings(world, pos)
        }
        Block::Hopper { .. } if !old.is_same_kind(state) => update_hopper(world, pos, state),
        _ => {}
    }
}

pub(crate) fn on_remove(world: &mut impl RedstoneWorld, pos: BlockPos, state: Block, new: Block) {
    let replaced = !new.is_same_kind(state);
    match state {
        Block::RedstoneWire(wire) => wire::on_remove(world, pos, wire, new),
        Block::Repeater(repeater) if replaced => {
            diode::update_neighbors_in_front(world, pos, repeater.facing.block_face())
        }
        Block::Comparator(comparator) if replaced => {
            diode::update_neighbors_in_front(world, pos, comparator.facing.block_face())
        }
        Block::RedstoneTorch { .. } | Block::RedstoneWallTorch { .. } => {
            torch::update_surroundings(world, pos)
        }
        _ if replaced && let Some((face, facing, true)) = attached::attachment(state) => {
            attached::update_neighbors(world, pos, face, facing)
        }
        Block::Barrel { .. } | Block::Furnace { .. } | Block::Hopper { .. } if replaced => {
            update::update_neighbors_for_output_signal(world, pos)
        }
        _ if replaced && state.pressure_plate_powered() == Some(true) => {
            plate::update_neighbors(world, pos)
        }
        _ => {}
    }
}

/// Returns true if the action was handled
pub fn use_block(world: &mut impl RedstoneWorld, pos: BlockPos) -> bool {
    let state = world.block(pos);
    match state {
        Block::RedstoneWire(wire) => wire::use_block(world, pos, wire),
        Block::Repeater(repeater) => {
            repeater::use_block(world, pos, repeater);
            true
        }
        Block::Comparator(comparator) => {
            comparator::use_block(world, pos, comparator);
            true
        }
        Block::Lever {
            face,
            facing,
            powered,
        } => {
            attached::pull_lever(world, pos, face, facing, powered);
            true
        }
        _ if let Some(button) = state.button() => {
            if !button.powered {
                attached::press_button(world, pos, state);
            }
            true
        }
        Block::NoteBlock {
            instrument,
            note,
            powered,
        } => {
            noteblock::use_block(world, pos, instrument, note, powered);
            true
        }
        _ => false,
    }
}

pub fn place_block(world: &mut impl RedstoneWorld, pos: BlockPos, state: Block) {
    if update::set_block(world, pos, state, Notify::All) && diode::is_diode(world.block(pos)) {
        update::neighbor_changed_at(world, pos);
    }
}

pub fn state_for_placement(world: &impl BlockView, pos: BlockPos, state: Block) -> Block {
    match state {
        Block::RedstoneWire(_) => Block::RedstoneWire(wire::state_for_placement(world, pos)),
        Block::Repeater(repeater) => {
            Block::Repeater(repeater::state_for_placement(world, pos, repeater))
        }
        Block::RedstoneLamp { .. } => Block::RedstoneLamp {
            lit: signal::has_neighbor_signal(world, pos),
        },
        Block::IronTrapdoor {
            facing,
            half,
            waterlogged,
            ..
        } => {
            let powered = signal::has_neighbor_signal(world, pos);
            Block::IronTrapdoor {
                facing,
                half,
                open: powered,
                powered,
                waterlogged,
            }
        }
        Block::NoteBlock { note, powered, .. } => {
            noteblock::state_for_placement(world, pos, note, powered)
        }
        _ => state,
    }
}

pub fn can_survive(world: &impl BlockView, pos: BlockPos, state: Block) -> bool {
    match state {
        Block::RedstoneWire(_) => wire::can_survive(world, pos),
        Block::Repeater(_) | Block::Comparator(_) => diode::can_survive(world, pos),
        Block::RedstoneTorch { .. } | Block::RedstoneWallTorch { .. } => {
            torch::can_survive(world, pos, state)
        }
        _ if let Some((face, facing, _)) = attached::attachment(state) => {
            attached::can_survive(world, pos, face, facing)
        }
        Block::TripwireHook { facing, .. } => world
            .block(pos.offset(facing.opposite().block_face()))
            .is_face_sturdy(facing.block_face()),
        _ if state.pressure_plate_powered().is_some() => plate::can_survive(world, pos),
        _ if state.is_sign() => supports_sign(world.block(pos.offset(BlockFace::Bottom))),
        _ => match state.get_wall_sign_facing() {
            Some(facing) => supports_sign(world.block(pos.offset(facing.opposite().block_face()))),
            None => true,
        },
    }
}

fn update_hopper(world: &mut impl RedstoneWorld, pos: BlockPos, state: Block) {
    let Block::Hopper { enabled, facing } = state else {
        return;
    };
    let should_be_enabled = !signal::has_neighbor_signal(world, pos);
    if should_be_enabled != enabled {
        let toggled = Block::Hopper {
            enabled: should_be_enabled,
            facing,
        };
        update::set_block(world, pos, toggled, Notify::Shapes);
    }
}

fn support_face(state: Block) -> Option<BlockFace> {
    match state {
        Block::RedstoneTorch { .. } | Block::RedstoneWallTorch { .. } => {
            Some(torch::support_face(state))
        }
        Block::TripwireHook { facing, .. } => Some(facing.opposite().block_face()),
        _ if let Some((face, facing, _)) = attached::attachment(state) => {
            Some(attached::connected_direction(face, facing).opposite())
        }
        _ if state.is_sign() || state.pressure_plate_powered().is_some() => Some(BlockFace::Bottom),
        _ => state
            .get_wall_sign_facing()
            .map(|facing| facing.opposite().block_face()),
    }
}

fn supports_sign(state: Block) -> bool {
    state.is_full_cube()
        || matches!(
            state,
            Block::SmoothStoneSlab { .. }
                | Block::QuartzSlab { .. }
                | Block::IronTrapdoor { .. }
                | Block::Hopper { .. }
                | Block::Composter { .. }
                | Block::Cauldron
                | Block::WaterCauldron { .. }
                | Block::EndPortalFrame { .. }
                | Block::Cake { bites: 0 }
        )
}
