use crate::block_entities::BlockEntity;
use crate::blocks::Block;
use crate::region::Region;
use crate::BlockPos;

pub trait BlockView {
    fn block(&self, pos: BlockPos) -> Block;

    fn block_entity(&self, pos: BlockPos) -> Option<&BlockEntity>;

    /// Covers every block in `region` that is not air, and may cover air as well.
    fn occupied_regions(&self, region: Region) -> Vec<Region> {
        vec![region]
    }
}

pub trait BlockStore: BlockView {
    /// Sets the block at `pos`.
    /// This function may have side effects such as sending update block packets to the player.
    /// Returns true if the block was changed.
    fn set_block(&mut self, pos: BlockPos, block: Block) -> bool;

    fn set_block_entity(&mut self, pos: BlockPos, block_entity: Option<BlockEntity>);
}
