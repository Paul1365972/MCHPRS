use crate::block_entities::BlockEntity;
use crate::blocks::Block;
use crate::region::Region;
use crate::BlockPos;

pub trait BlockView {
    fn block(&self, position: BlockPos) -> Block;

    fn block_entity(&self, position: BlockPos) -> Option<&BlockEntity>;

    /// Covers every block in `region` that is not air, and may cover air as well.
    fn occupied_regions(&self, region: Region) -> Vec<Region> {
        vec![region]
    }
}

pub trait BlockStore: BlockView {
    fn set_block(&mut self, position: BlockPos, block: Block) -> bool;

    fn set_block_entity(&mut self, position: BlockPos, block_entity: Option<BlockEntity>);
}
