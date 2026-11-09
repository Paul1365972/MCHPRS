use crate::sound::Sound;
use crate::update::{self, Notify};
use crate::{signal, RedstoneWorld};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::{Block, Instrument};
use mchprs_blocks::{BlockFace, BlockPos};

pub fn is_unblocked(world: &impl BlockView, pos: BlockPos) -> bool {
    matches!(world.block(pos.offset(BlockFace::Top)), Block::Air)
}

fn instrument(world: &impl BlockView, pos: BlockPos) -> Instrument {
    Instrument::from_block_below(world.block(pos.offset(BlockFace::Bottom)))
}

pub(crate) fn state_for_placement(
    world: &impl BlockView,
    pos: BlockPos,
    note: u8,
    powered: bool,
) -> Block {
    Block::NoteBlock {
        instrument: instrument(world, pos),
        note,
        powered,
    }
}

pub(crate) fn neighbor_changed(
    world: &mut impl RedstoneWorld,
    pos: BlockPos,
    instrument: Instrument,
    note: u8,
    powered: bool,
) {
    let should_be_powered = signal::has_neighbor_signal(world, pos);
    if should_be_powered == powered {
        return;
    }
    if should_be_powered {
        play_note(world, pos, instrument, note);
    }
    let state = Block::NoteBlock {
        instrument,
        note,
        powered: should_be_powered,
    };
    update::set_block(world, pos, state, Notify::All);
}

pub(crate) fn use_block(
    world: &mut impl RedstoneWorld,
    pos: BlockPos,
    instrument: Instrument,
    note: u8,
    powered: bool,
) {
    let note = (note + 1) % 25;
    let state = Block::NoteBlock {
        instrument,
        note,
        powered,
    };
    update::set_block(world, pos, state, Notify::All);
    play_note(world, pos, instrument, note);
}

fn play_note(world: &mut impl RedstoneWorld, pos: BlockPos, instrument: Instrument, note: u8) {
    if is_unblocked(world, pos) {
        world.play_sound(pos, Sound::note(instrument, note));
    }
}
