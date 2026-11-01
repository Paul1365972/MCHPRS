use crate::tick_queue::{run_scheduled_ticks, TickEntry, TickPriority, TickQueue};
use crate::RedstoneWorld;
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;

#[derive(Debug)]
pub struct Interpreter<B: BlockStore> {
    blocks: B,
    ticks: TickQueue,
}

impl<B: BlockStore> Interpreter<B> {
    pub fn new(blocks: B, pending: Vec<TickEntry>) -> Interpreter<B> {
        Interpreter {
            blocks,
            ticks: TickQueue::new(pending),
        }
    }

    pub fn tick(&mut self) {
        run_scheduled_ticks(self, |interpreter| &mut interpreter.ticks);
    }

    pub fn use_block(&mut self, pos: BlockPos) {
        crate::on_use(self.block(pos), self, pos);
    }

    pub fn blocks(&self) -> &B {
        &self.blocks
    }

    pub fn pending_ticks(&self) -> &[TickEntry] {
        self.ticks.entries()
    }
}

impl<B: BlockStore> BlockView for Interpreter<B> {
    fn block(&self, pos: BlockPos) -> Block {
        self.blocks.block(pos)
    }

    fn block_entity(&self, pos: BlockPos) -> Option<&BlockEntity> {
        self.blocks.block_entity(pos)
    }

    fn occupied_regions(&self, region: Region) -> Vec<Region> {
        self.blocks.occupied_regions(region)
    }
}

impl<B: BlockStore> BlockStore for Interpreter<B> {
    fn set_block(&mut self, pos: BlockPos, block: Block) -> bool {
        self.blocks.set_block(pos, block)
    }

    fn set_block_entity(&mut self, pos: BlockPos, block_entity: Option<BlockEntity>) {
        self.blocks.set_block_entity(pos, block_entity);
    }
}

impl<B: BlockStore> RedstoneWorld for Interpreter<B> {
    fn schedule_tick(&mut self, pos: BlockPos, delay: u32, priority: TickPriority) {
        let block = self.blocks.block(pos);
        self.ticks.schedule(pos, delay, priority, block);
    }

    fn pending_tick_at(&self, pos: BlockPos) -> bool {
        self.ticks.is_pending(pos, self.blocks.block(pos))
    }
}
