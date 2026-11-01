use crate::block_entities::BlockEntity;
use crate::block_store::{BlockStore, BlockView};
use crate::blocks::Block;
use crate::region::Region;
use crate::BlockPos;
use std::collections::BTreeMap;

#[derive(Clone, Debug)]
pub struct BlockVolume {
    region: Region,
    block_states: Vec<u16>,
    block_entities: BTreeMap<usize, BlockEntity>,
}

impl BlockVolume {
    pub fn empty(region: Region) -> BlockVolume {
        let [length_x, length_y, length_z] = lengths(region);
        BlockVolume {
            region,
            block_states: vec![0; length_x * length_y * length_z],
            block_entities: BTreeMap::new(),
        }
    }

    pub fn region(&self) -> Region {
        self.region
    }

    pub fn translated(self, offset: BlockPos) -> BlockVolume {
        BlockVolume {
            region: self.region.translated(offset),
            ..self
        }
    }

    pub fn block_entities(&self) -> impl Iterator<Item = (BlockPos, &BlockEntity)> {
        self.block_entities
            .iter()
            .map(|(index, block_entity)| (self.position(*index), block_entity))
    }

    fn index(&self, position: BlockPos) -> Option<usize> {
        if !self.region.contains(position) {
            return None;
        }
        let [length_x, _, length_z] = lengths(self.region);
        let min = self.region.min();
        let x = offset(min.x, position.x);
        let y = offset(min.y, position.y);
        let z = offset(min.z, position.z);
        Some((y * length_z + z) * length_x + x)
    }

    fn position(&self, index: usize) -> BlockPos {
        let [length_x, _, length_z] = lengths(self.region);
        let min = self.region.min();
        BlockPos::new(
            coordinate(min.x, index % length_x),
            coordinate(min.y, index / length_x / length_z),
            coordinate(min.z, index / length_x % length_z),
        )
    }
}

fn lengths(region: Region) -> [usize; 3] {
    let length = |min: i32, max: i32| {
        usize::try_from(i64::from(max) - i64::from(min) + 1).unwrap_or_default()
    };
    [
        length(region.min().x, region.max().x),
        length(region.min().y, region.max().y),
        length(region.min().z, region.max().z),
    ]
}

fn offset(min: i32, coordinate: i32) -> usize {
    (i64::from(coordinate) - i64::from(min)) as usize
}

fn coordinate(min: i32, offset: usize) -> i32 {
    (i64::from(min) + offset as i64) as i32
}

impl BlockView for BlockVolume {
    fn block(&self, position: BlockPos) -> Block {
        self.index(position).map_or(Block::Air, |index| {
            Block::from_id(u32::from(self.block_states[index]))
        })
    }

    fn block_entity(&self, position: BlockPos) -> Option<&BlockEntity> {
        self.block_entities.get(&self.index(position)?)
    }

    fn occupied_regions(&self, region: Region) -> Vec<Region> {
        self.region.intersection(region).into_iter().collect()
    }
}

impl BlockStore for BlockVolume {
    fn set_block(&mut self, position: BlockPos, block: Block) -> bool {
        let Some(index) = self.index(position) else {
            return false;
        };
        let block_state = u16::try_from(block.get_id()).unwrap_or_default();
        let changed = self.block_states[index] != block_state;
        self.block_states[index] = block_state;
        changed
    }

    fn set_block_entity(&mut self, position: BlockPos, block_entity: Option<BlockEntity>) {
        let Some(index) = self.index(position) else {
            return;
        };
        match block_entity {
            Some(block_entity) => self.block_entities.insert(index, block_entity),
            None => self.block_entities.remove(&index),
        };
    }
}
