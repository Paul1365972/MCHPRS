use crate::block_changes::BlockChanges;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_redstone::tick_queue::PendingTick;
use mchprs_redstone::{update_region, RedstoneWorld};

/// Brings a world that holds what was shown to the state a simulation reached.
#[derive(Debug, Clone, PartialEq)]
pub struct WriteBack {
    pub changes: BlockChanges,
    pub ticks: Vec<(BlockPos, PendingTick)>,
    /// What was compiled. A block that the simulation left out, such as plain wire, is updated
    /// from its neighbors.
    pub region: Region,
}

impl WriteBack {
    pub fn apply(&self, world: &mut impl RedstoneWorld) {
        self.changes.apply(world);
        for &(pos, tick) in &self.ticks {
            let block = world.block(pos);
            world.schedule_tick(pos, tick.delay, tick.priority, block);
        }
        update_region(world, self.region);
    }
}
