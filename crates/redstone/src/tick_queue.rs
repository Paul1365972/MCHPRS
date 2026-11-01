use crate::RedstoneWorld;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::BlockPos;
use serde::{Deserialize, Serialize};
use std::mem;

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum TickPriority {
    Highest = 0,
    Higher = 1,
    High = 2,
    Normal = 3,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TickEntry {
    pub ticks_left: u32,
    pub priority: TickPriority,
    pub pos: BlockPos,
    pub block: Block,
}

impl TickEntry {
    pub fn new(pos: BlockPos, delay: u32, priority: TickPriority, block: Block) -> TickEntry {
        TickEntry {
            ticks_left: delay,
            priority,
            pos,
            block,
        }
    }

    pub fn is_for(&self, block: Block) -> bool {
        mem::discriminant(&self.block) == mem::discriminant(&block)
    }
}

#[derive(Debug, Clone, Default)]
pub struct TickQueue {
    entries: Vec<TickEntry>,
}

impl TickQueue {
    pub fn new(entries: Vec<TickEntry>) -> TickQueue {
        TickQueue { entries }
    }

    /// Schedules a tick unless `block` has one in a later tick, like vanilla `LevelChunkTicks`
    pub fn schedule(&mut self, pos: BlockPos, delay: u32, priority: TickPriority, block: Block) {
        let scheduled = self
            .entries
            .iter()
            .any(|entry| entry.pos == pos && entry.ticks_left > 0 && entry.is_for(block));
        if !scheduled {
            self.entries
                .push(TickEntry::new(pos, delay, priority, block));
        }
    }

    pub fn is_pending(&self, pos: BlockPos, block: Block) -> bool {
        self.entries
            .iter()
            .any(|entry| entry.pos == pos && entry.is_for(block))
    }

    pub fn entries(&self) -> &[TickEntry] {
        &self.entries
    }

    pub fn into_entries(self) -> Vec<TickEntry> {
        self.entries
    }

    fn advance(&mut self) {
        self.entries
            .sort_by_key(|entry| (entry.ticks_left, entry.priority));
        for entry in &mut self.entries {
            entry.ticks_left = entry.ticks_left.saturating_sub(1);
        }
    }

    fn pop_due(&mut self) -> Option<TickEntry> {
        if self.entries.first()?.ticks_left > 0 {
            return None;
        }
        Some(self.entries.remove(0))
    }
}

pub fn run_scheduled_ticks<W: RedstoneWorld>(
    world: &mut W,
    tick_queue: impl Fn(&mut W) -> &mut TickQueue,
) {
    tick_queue(world).advance();
    while let Some(entry) = tick_queue(world).pop_due() {
        let block = world.block(entry.pos);
        if entry.is_for(block) {
            crate::tick(block, world, entry.pos);
        }
    }
}
