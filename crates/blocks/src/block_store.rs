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

    fn occupied_positions(&self, region: Region) -> impl Iterator<Item = BlockPos> + use<Self>
    where
        Self: Sized,
    {
        self.occupied_regions(region)
            .into_iter()
            .flat_map(Region::positions)
    }
}

impl<T: BlockView + ?Sized> BlockView for &T {
    fn block(&self, pos: BlockPos) -> Block {
        (**self).block(pos)
    }

    fn block_entity(&self, pos: BlockPos) -> Option<&BlockEntity> {
        (**self).block_entity(pos)
    }

    fn occupied_regions(&self, region: Region) -> Vec<Region> {
        (**self).occupied_regions(region)
    }
}

pub trait BlockStore: BlockView {
    /// Returns whether the block changed.
    fn set_block(&mut self, pos: BlockPos, block: Block) -> bool;

    fn set_block_entity(&mut self, pos: BlockPos, block_entity: Option<BlockEntity>);
}
