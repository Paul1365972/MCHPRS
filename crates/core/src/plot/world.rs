use super::*;
use std::collections::HashMap;

pub struct PlotWorld {
    pub x: i32,
    pub z: i32,
    pub chunks: Vec<Chunk>,
    pub(super) redstone: Redstone,
    pub(super) entity_boxes: Vec<Aabb>,
    pub packet_senders: HashMap<EntityId, PlayerPacketSender>,
    pub(super) world_send_rate: WorldSendRate,
    pub(super) pending_block_entities: FxHashMap<BlockPos, CBlockEntityData>,
    pub(super) pending_sounds: FxHashMap<BlockPos, Sound>,
}

impl PlotWorld {
    pub(super) fn press_touched_plates(&mut self) -> bool {
        let boxes = self.entity_boxes.clone();
        let mut pressed = false;
        for pos in boxes.iter().flat_map(Aabb::block_positions) {
            pressed |= plate::entity_inside(self, pos);
        }
        pressed
    }

    fn get_chunk_index_for_chunk(&self, chunk_x: i32, chunk_z: i32) -> Option<usize> {
        let local_x = chunk_x - self.x * PLOT_WIDTH;
        let local_z = chunk_z - self.z * PLOT_WIDTH;
        if !(0..PLOT_WIDTH).contains(&local_x) || !(0..PLOT_WIDTH).contains(&local_z) {
            return None;
        }
        Some((local_x * PLOT_WIDTH + local_z) as usize)
    }

    fn get_chunk_index_for_block(&self, block_x: i32, block_z: i32) -> Option<usize> {
        let chunk_x = (block_x - (self.x * PLOT_BLOCK_WIDTH)) >> 4;
        let chunk_z = (block_z - (self.z * PLOT_BLOCK_WIDTH)) >> 4;
        if !(0..PLOT_WIDTH).contains(&chunk_x) || !(0..PLOT_WIDTH).contains(&chunk_z) {
            return None;
        }
        Some(((chunk_x << PLOT_SCALE) + chunk_z).unsigned_abs() as usize)
    }

    pub fn region(&self) -> Region {
        const W: i32 = PLOT_BLOCK_WIDTH;
        Region::new(
            BlockPos::new(self.x * W, 0, self.z * W),
            BlockPos::new(
                (self.x + 1) * W - 1,
                PLOT_BLOCK_HEIGHT - 1,
                (self.z + 1) * W - 1,
            ),
        )
    }

    pub fn chunk(&self, x: i32, z: i32) -> Option<&Chunk> {
        self.chunks.get(self.get_chunk_index_for_chunk(x, z)?)
    }

    fn sign_data(&self, pos: BlockPos) -> Option<CBlockEntityData> {
        let blank;
        let sign = match self.block_entity(pos) {
            Some(sign @ BlockEntity::Sign(_)) => sign,
            _ => {
                blank = BlockEntity::Sign(Box::default());
                &blank
            }
        };
        let nbt = sign.to_nbt(true)?;
        Some(CBlockEntityData {
            x: pos.x,
            y: pos.y,
            z: pos.z,
            ty: sign.ty(),
            nbt,
        })
    }
}

impl BlockView for PlotWorld {
    fn block(&self, pos: BlockPos) -> Block {
        let Some(chunk_index) = self.get_chunk_index_for_block(pos.x, pos.z) else {
            return Block::Air;
        };
        let chunk = &self.chunks[chunk_index];
        Block::from_id(chunk.get_block((pos.x & 0xF) as u32, pos.y as u32, (pos.z & 0xF) as u32))
            .expect("plot chunks hold known block states")
    }

    fn block_entity(&self, pos: BlockPos) -> Option<&BlockEntity> {
        let chunk_index = self.get_chunk_index_for_block(pos.x, pos.z)?;
        let chunk = &self.chunks[chunk_index];
        chunk.get_block_entity(BlockPos::new(pos.x & 0xF, pos.y, pos.z & 0xF))
    }

    fn occupied_regions(&self, region: Region) -> Vec<Region> {
        occupied_regions(region, |x, z| self.chunk(x, z))
    }
}

impl BlockStore for PlotWorld {
    /// Sets a block in storage. Returns true if a block was changed.
    fn set_block(&mut self, pos: BlockPos, block: Block) -> bool {
        let chunk_index = match self.get_chunk_index_for_block(pos.x, pos.z) {
            Some(idx) => idx,
            None => return false,
        };

        // Check to see if block is within height limit
        if pos.y >= PLOT_BLOCK_HEIGHT || pos.y < 0 {
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
        let Some(block_entity) = block_entity else {
            let had_sign = matches!(self.block_entity(pos), Some(BlockEntity::Sign(_)));
            self.pending_block_entities.remove(&pos);
            let chunk_index = match self.get_chunk_index_for_block(pos.x, pos.z) {
                Some(idx) => idx,
                None => return,
            };
            let chunk = &mut self.chunks[chunk_index];
            chunk.delete_block_entity(BlockPos::new(pos.x & 0xF, pos.y, pos.z & 0xF));
            let block = self.block(pos);
            if had_sign
                && block.is_any_sign()
                && let Some(blank) = self.sign_data(pos)
            {
                self.pending_block_entities.insert(pos, blank);
            }
            return;
        };
        if !(0..PLOT_BLOCK_HEIGHT).contains(&pos.y) {
            return;
        }
        let chunk_index = match self.get_chunk_index_for_block(pos.x, pos.z) {
            Some(idx) => idx,
            None => return,
        };
        if let Some(nbt) = block_entity.to_nbt(true) {
            let block_entity_data = CBlockEntityData {
                x: pos.x,
                y: pos.y,
                z: pos.z,
                // For now the only nbt we send to the client is sign data
                ty: block_entity.ty(),
                nbt,
            };
            self.pending_block_entities.insert(pos, block_entity_data);
        } else {
            self.pending_block_entities.remove(&pos);
        }
        let chunk = &mut self.chunks[chunk_index];
        chunk.set_block_entity(BlockPos::new(pos.x & 0xF, pos.y, pos.z & 0xF), block_entity);
    }
}

impl RedstoneWorld for PlotWorld {
    fn redstone(&mut self) -> &mut Redstone {
        &mut self.redstone
    }
    fn has_entity_in(&self, area: Aabb) -> bool {
        self.entity_boxes
            .iter()
            .any(|entity| entity.intersects(&area))
    }

    fn play_sound(&mut self, pos: BlockPos, sound: Sound) {
        if self.world_send_rate.0 != 0.0 {
            self.pending_sounds.insert(pos, sound);
        }
    }
}

impl PlotWorld {
    pub fn flush_block_changes(&mut self) {
        for packet in self.chunks.iter_mut().flat_map(encode_block_changes) {
            for packet_sender in self.packet_senders.values() {
                packet_sender.send_packet(&packet);
            }
        }
        for (_, block_entity) in self.pending_block_entities.drain() {
            let encoded = block_entity.encode();
            for packet_sender in self.packet_senders.values() {
                packet_sender.send_packet(&encoded);
            }
        }
    }
}
