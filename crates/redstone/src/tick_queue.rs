use crate::{behavior, RedstoneWorld};
use mchprs_blocks::blocks::Block;
use mchprs_blocks::BlockPos;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::fmt;
use std::str::FromStr;

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum TickPriority {
    Highest = 0,
    Higher = 1,
    High = 2,
    Normal = 3,
}

impl TickPriority {
    pub const ALL: [TickPriority; 4] = [
        TickPriority::Highest,
        TickPriority::Higher,
        TickPriority::High,
        TickPriority::Normal,
    ];

    fn name(self) -> &'static str {
        match self {
            TickPriority::Highest => "highest",
            TickPriority::Higher => "higher",
            TickPriority::High => "high",
            TickPriority::Normal => "normal",
        }
    }
}

impl fmt::Display for TickPriority {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

impl FromStr for TickPriority {
    type Err = UnknownPriority;

    fn from_str(name: &str) -> Result<TickPriority, UnknownPriority> {
        TickPriority::ALL
            .into_iter()
            .find(|priority| priority.name() == name)
            .ok_or_else(|| UnknownPriority(name.to_owned()))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("unknown tick priority `{0}`")]
pub struct UnknownPriority(String);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct PendingTick {
    pub delay: u32,
    pub priority: TickPriority,
}

impl fmt::Display for PendingTick {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} {}", self.delay, self.priority)
    }
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
        self.block.is_same_kind(block)
    }
}

#[derive(Debug, Clone, Default)]
pub struct TickQueue {
    entries: Vec<TickEntry>,
    due: VecDeque<TickEntry>,
}

impl TickQueue {
    pub fn new(entries: Vec<TickEntry>) -> TickQueue {
        TickQueue {
            entries,
            due: VecDeque::new(),
        }
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
        let mut entries = self.due.iter().chain(&self.entries);
        entries.any(|entry| entry.pos == pos && entry.is_for(block))
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
        let due_len = self.entries.partition_point(|entry| entry.ticks_left == 0);
        self.due.extend(self.entries.drain(..due_len));
    }
}

pub fn run_scheduled_ticks(world: &mut impl RedstoneWorld) {
    world.redstone().ticks.advance();
    while let Some(entry) = world.redstone().ticks.due.pop_front() {
        let block = world.block(entry.pos);
        if entry.is_for(block) {
            behavior::tick(world, entry.pos, block);
        }
    }
}
