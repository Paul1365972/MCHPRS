use crate::{
    has_tick_entry, occupied_regions, schedule_tick_entry, storage::Chunk, TickEntry, TickPriority,
    World,
};
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;

/// A reference world implementation used for testing.
#[derive(Clone)]
pub struct TestWorld {
    pub chunks: Vec<Chunk>,
    pub to_be_ticked: Vec<TickEntry>,
    /// The size in chunks on the X azis.
    pub x_size: i32,
    /// The size in chunk sections on the Y azis.
    pub y_size: i32,
    /// The size in chunks on the Z azis.
    pub z_size: i32,
}

impl TestWorld {
    /// Create a new square world for testing with a size in chunks
    pub fn new(x_size: i32, y_size: i32, z_size: i32) -> TestWorld {
        let mut chunks = Vec::new();
        for x in 0..x_size {
            for z in 0..z_size {
                chunks.push(Chunk::empty(x, z, y_size as usize));
            }
        }
        TestWorld {
            chunks,
            to_be_ticked: Vec::new(),
            x_size,
            y_size,
            z_size,
        }
    }

    fn get_chunk_index_for_block(&self, block_x: i32, block_z: i32) -> Option<usize> {
        let chunk_x = block_x >> 4;
        let chunk_z = block_z >> 4;
        if chunk_x >= self.x_size || chunk_z >= self.z_size || chunk_x < 0 || chunk_z < 0 {
            return None;
        }
        Some(((chunk_x * self.z_size) + chunk_z).unsigned_abs() as usize)
    }
}

impl BlockView for TestWorld {
    fn block(&self, pos: BlockPos) -> Block {
        let chunk_index = match self.get_chunk_index_for_block(pos.x, pos.z) {
            Some(idx) => idx,
            None => return Block::Air,
        };
        let chunk = &self.chunks[chunk_index];
        Block::from_id(chunk.get_block((pos.x & 0xF) as u32, pos.y as u32, (pos.z & 0xF) as u32))
    }

    fn block_entity(&self, pos: BlockPos) -> Option<&BlockEntity> {
        let chunk_index = self.get_chunk_index_for_block(pos.x, pos.z)?;
        let chunk = &self.chunks[chunk_index];
        chunk.get_block_entity(BlockPos::new(pos.x & 0xF, pos.y, pos.z & 0xF))
    }

    fn occupied_regions(&self, region: Region) -> Vec<Region> {
        occupied_regions(region, |x, z| {
            self.chunks
                .iter()
                .find(|chunk| chunk.x == x && chunk.z == z)
        })
    }
}

impl BlockStore for TestWorld {
    fn set_block(&mut self, pos: BlockPos, block: Block) -> bool {
        let chunk_index = match self.get_chunk_index_for_block(pos.x, pos.z) {
            Some(idx) => idx,
            None => return false,
        };

        // Check to see if block is within height limit
        if pos.y >= self.y_size * 16 || pos.y < 0 {
            return false;
        }

        let chunk = &mut self.chunks[chunk_index];
        chunk.set_block(
            (pos.x & 0xF) as u32,
            pos.y as u32,
            (pos.z & 0xF) as u32,
            block.get_id(),
        )
    }

    fn set_block_entity(&mut self, pos: BlockPos, block_entity: Option<BlockEntity>) {
        let chunk_index = match self.get_chunk_index_for_block(pos.x, pos.z) {
            Some(idx) => idx,
            None => return,
        };
        let chunk = &mut self.chunks[chunk_index];
        let pos = BlockPos::new(pos.x & 0xF, pos.y, pos.z & 0xF);
        match block_entity {
            Some(block_entity) => chunk.set_block_entity(pos, block_entity),
            None => chunk.delete_block_entity(pos),
        }
    }
}

impl World for TestWorld {
    fn schedule_tick(&mut self, pos: BlockPos, delay: u32, priority: TickPriority) {
        let block = self.block(pos);
        schedule_tick_entry(&mut self.to_be_ticked, block, pos, delay, priority);
    }

    fn pending_tick_at(&mut self, pos: BlockPos) -> bool {
        has_tick_entry(&self.to_be_ticked, self.block(pos), pos)
    }
}
