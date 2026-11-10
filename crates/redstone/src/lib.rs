pub mod attached;
mod behavior;
pub mod comparator;
pub mod diode;
pub mod input;
pub mod interpreter;
pub mod noteblock;
pub mod plate;
pub mod repeater;
mod signal;
pub mod sound;
pub mod tick_queue;
pub mod torch;
mod update;
pub mod wire;

use mchprs_blocks::block_store::BlockStore;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::{BlockDirection, BlockFace, BlockPos};
use sound::Sound;
use tick_queue::{TickPriority, TickQueue};

pub use behavior::{
    can_survive, place_block, state_for_placement, use_block, LAMP_OFF_DELAY_TICKS,
};
pub use signal::{emitters, provides_weak_power, Dust, Emitter};
pub use update::{set_block, update_neighbors_for_output_signal, update_region, Notify};

/// Vanilla's `Direction.values()` order
const DIRECTIONS: [BlockFace; 6] = [
    BlockFace::Bottom,
    BlockFace::Top,
    BlockFace::North,
    BlockFace::South,
    BlockFace::West,
    BlockFace::East,
];

/// Vanilla's `Direction.Plane.HORIZONTAL` order
pub const HORIZONTAL: [BlockDirection; 4] = [
    BlockDirection::North,
    BlockDirection::East,
    BlockDirection::South,
    BlockDirection::West,
];

#[derive(Debug)]
pub struct Redstone {
    pub ticks: TickQueue,
    updater: update::NeighborUpdater,
    block_events: Vec<BlockPos>,
}

impl Redstone {
    pub fn new(ticks: TickQueue) -> Redstone {
        Redstone {
            ticks,
            updater: Default::default(),
            block_events: Vec::new(),
        }
    }

    pub fn is_idle(&self) -> bool {
        self.ticks.entries().is_empty() && self.block_events.is_empty()
    }
}

pub trait RedstoneWorld: BlockStore {
    fn redstone(&mut self) -> &mut Redstone;

    fn play_sound(&mut self, pos: BlockPos, sound: Sound);

    fn schedule_tick(&mut self, pos: BlockPos, delay: u32, priority: TickPriority, block: Block) {
        self.redstone().ticks.schedule(pos, delay, priority, block);
    }
}

/// Updates every block in `region`, then runs at most `limit` ticks until no tick or block event
/// is pending, and returns how many it ran if it got there.
pub fn settle(world: &mut impl RedstoneWorld, region: Region, limit: u64) -> Option<u64> {
    update_region(world, region);
    for ticks in 0..limit {
        if world.redstone().is_idle() {
            return Some(ticks);
        }
        tick_queue::run_scheduled_ticks(world);
    }
    world.redstone().is_idle().then_some(limit)
}
