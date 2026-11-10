use mchprs_blocks::block_state;
use mchprs_blocks::block_store::BlockStore;
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::blocks::Instrument;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_redstone::interpreter::Interpreter;
use mchprs_redstone::sound::Sound;

const LEVER: BlockPos = BlockPos::new(0, 1, 0);
const NOTE_BLOCK: BlockPos = BlockPos::new(1, 1, 0);

fn lever_beside_note_block() -> Interpreter<BlockVolume> {
    let region = Region::new(BlockPos::new(-1, 0, -1), BlockPos::new(2, 2, 1));
    let mut volume = BlockVolume::empty(region);
    let block = |state| {
        block_state::stored(state)
            .expect("a block state")
            .expect("a known block")
    };
    volume.set_block(BlockPos::new(0, 0, 0), block("minecraft:stone"));
    volume.set_block(
        LEVER,
        block("minecraft:lever[face=floor,facing=north,powered=false]"),
    );
    volume.set_block(
        NOTE_BLOCK,
        block("minecraft:note_block[instrument=harp,note=0,powered=false]"),
    );
    Interpreter::new(volume, Vec::new())
}

fn notes(world: &mut Interpreter<BlockVolume>) -> Vec<Sound> {
    let sounds = world.take_sounds().into_iter();
    let notes = sounds.filter(|&(pos, _)| pos == NOTE_BLOCK);
    notes.map(|(_, sound)| sound).collect()
}

#[test]
fn three_toggles_between_ticks_play_one_note() {
    let mut world = lever_beside_note_block();
    for _ in 0..3 {
        world.use_block(LEVER);
    }
    assert_eq!(notes(&mut world), []);
    world.tick();
    assert_eq!(notes(&mut world), [Sound::note(Instrument::Harp, 0)]);
}
