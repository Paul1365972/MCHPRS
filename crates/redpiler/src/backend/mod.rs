pub mod direct;

use crate::engine::Input;
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::blocks::{Block, Instrument};
use mchprs_blocks::BlockPos;
use mchprs_redstone::noteblock;
use mchprs_world::{SendRate, TickEntry, TickRate, World};
use std::error::Error;
use std::fmt::{self, Display, Formatter};
use std::time::Duration;

#[derive(Default, Debug)]
pub struct Batch {
    pub blocks: Vec<(BlockPos, Block)>,
    pub block_entities: Vec<(BlockPos, BlockEntity)>,
    pub notes: Vec<(BlockPos, Instrument, u8)>,
    pub ticks_completed: u64,
    pub ticks_owed: u64,
}

impl Batch {
    pub fn changes_world(&self) -> bool {
        !(self.blocks.is_empty() && self.block_entities.is_empty() && self.notes.is_empty())
    }

    pub fn append(&mut self, later: Batch) {
        self.blocks.extend(later.blocks);
        self.block_entities.extend(later.block_entities);
        self.notes.extend(later.notes);
        self.ticks_completed += later.ticks_completed;
        self.ticks_owed = later.ticks_owed;
    }

    pub fn apply(self, world: &mut impl World) {
        for (pos, block) in self.blocks {
            world.set_block(pos, block);
        }
        for (pos, block_entity) in self.block_entities {
            world.set_block_entity(pos, block_entity);
        }
        for (pos, instrument, note) in self.notes {
            noteblock::play_note(world, pos, instrument, note);
        }
    }
}

#[derive(Debug)]
pub struct Snapshot {
    pub changes: Batch,
    pub pending_ticks: Vec<TickEntry>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BackendFailure;

impl Display for BackendFailure {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.write_str("the redpiler backend has failed")
    }
}

impl Error for BackendFailure {}

pub trait Backend: Send {
    fn set_tick_rate(&mut self, rate: TickRate);
    fn set_send_rate(&mut self, rate: SendRate);
    fn input(&mut self, pos: BlockPos, input: Input);
    fn next_batch(&mut self, timeout: Duration) -> Result<Batch, BackendFailure>;
    fn snapshot(&mut self) -> Result<Snapshot, BackendFailure>;
    fn stop(self: Box<Self>) -> Result<Snapshot, BackendFailure>;
}
