use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::{Block, Instrument};
use mchprs_blocks::{BlockFace, BlockPos};

pub fn is_noteblock_unblocked(world: &impl BlockView, pos: BlockPos) -> bool {
    matches!(world.block(pos.offset(BlockFace::Top)), Block::Air)
}

pub fn get_noteblock_instrument(world: &impl BlockView, pos: BlockPos) -> Instrument {
    Instrument::from_block_below(world.block(pos.offset(BlockFace::Bottom)))
}
