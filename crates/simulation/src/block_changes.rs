use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::BlockStore;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::BlockPos;
use mchprs_redstone::sound::Sound;
use mchprs_redstone::RedstoneWorld;

#[derive(Debug, Clone, Default, PartialEq)]
pub struct BlockChanges {
    pub blocks: Vec<(BlockPos, Block)>,
    pub block_entities: Vec<(BlockPos, Option<BlockEntity>)>,
    pub sounds: Vec<(BlockPos, Sound)>,
}

impl BlockChanges {
    pub fn append(&mut self, other: &mut BlockChanges) {
        self.blocks.append(&mut other.blocks);
        self.block_entities.append(&mut other.block_entities);
        self.sounds.append(&mut other.sounds);
    }

    pub fn apply(&self, world: &mut impl RedstoneWorld) {
        self.write(world);
        for &(pos, sound) in &self.sounds {
            world.play_sound(pos, sound);
        }
    }

    pub fn write(&self, blocks: &mut impl BlockStore) {
        for &(pos, block) in &self.blocks {
            blocks.set_block(pos, block);
        }
        for (pos, block_entity) in &self.block_entities {
            blocks.set_block_entity(*pos, block_entity.clone());
        }
    }
}

impl Extend<BlockChanges> for BlockChanges {
    fn extend<I: IntoIterator<Item = BlockChanges>>(&mut self, later: I) {
        for mut changes in later {
            self.append(&mut changes);
        }
    }
}
