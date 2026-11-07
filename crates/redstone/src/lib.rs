//! Vanilla 1.20.4's redstone: a settled circuit driven by its inputs runs exactly as in vanilla,
//! except that torches never burn out and observers stay inert. Placing and breaking blocks works
//! but may differ in details. Plots run on it whenever Redpiler isn't running.
//!
//! Time runs in redstone ticks, two game ticks each, so every vanilla delay is halved

mod attached;
mod behavior;
pub mod comparator;
mod diode;
pub mod interpreter;
pub mod noteblock;
pub mod pace;
pub mod plate;
mod repeater;
mod signal;
pub mod sound;
pub mod tick_queue;
mod torch;
mod update;
pub mod wire;

use mchprs_blocks::block_store::BlockStore;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::{BlockDirection, BlockFace, BlockPos};
use sound::Sound;
use tick_queue::{TickPriority, TickQueue};

pub use behavior::{can_survive, place_block, state_for_placement, use_block};
pub use diode::{diode_should_be_prioritized, is_diode};
pub use signal::{provides_strong_power, provides_weak_power};
pub use update::{
    neighbor_changed_at, set_block, update_neighbors_for_output_signal, update_region, Notify,
};

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
const HORIZONTAL: [BlockDirection; 4] = [
    BlockDirection::North,
    BlockDirection::East,
    BlockDirection::South,
    BlockDirection::West,
];

#[derive(Debug)]
pub struct Redstone {
    pub ticks: TickQueue,
    updater: update::NeighborUpdater,
}

impl Redstone {
    pub fn new(ticks: TickQueue) -> Redstone {
        Redstone {
            ticks,
            updater: Default::default(),
        }
    }
}

pub trait RedstoneWorld: BlockStore {
    fn redstone(&mut self) -> &mut Redstone;

    fn has_entity_in(&self, area: Aabb) -> bool;

    fn play_sound(&mut self, _pos: BlockPos, _sound: Sound) {}

    fn schedule_tick(&mut self, pos: BlockPos, delay: u32, priority: TickPriority, block: Block) {
        self.redstone().ticks.schedule(pos, delay, priority, block);
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Aabb {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

impl Aabb {
    pub fn intersects(&self, other: &Aabb) -> bool {
        (0..3).all(|axis| self.min[axis] < other.max[axis] && self.max[axis] > other.min[axis])
    }

    pub fn block_positions(&self) -> impl Iterator<Item = BlockPos> {
        // Vanilla's checkInsideBlocks shrinks the box by this much
        const MARGIN: f64 = 1.0e-7;
        let min = self.min.map(|coord| (coord + MARGIN).floor() as i32);
        let max = self.max.map(|coord| (coord - MARGIN).floor() as i32);
        (min[0]..=max[0]).flat_map(move |x| {
            (min[1]..=max[1])
                .flat_map(move |y| (min[2]..=max[2]).map(move |z| BlockPos::new(x, y, z)))
        })
    }

    fn offset(self, pos: BlockPos) -> Aabb {
        let delta = [pos.x as f64, pos.y as f64, pos.z as f64];
        Aabb {
            min: std::array::from_fn(|axis| self.min[axis] + delta[axis]),
            max: std::array::from_fn(|axis| self.max[axis] + delta[axis]),
        }
    }
}

/// Updates every block in `region`, then runs at most `limit` ticks until none is pending, and
/// returns how many it ran if it got there.
pub fn settle(world: &mut impl RedstoneWorld, region: Region, limit: u64) -> Option<u64> {
    update_region(world, region);
    for ticks in 0..limit {
        if world.redstone().ticks.entries().is_empty() {
            return Some(ticks);
        }
        tick_queue::run_scheduled_ticks(world);
    }
    world.redstone().ticks.entries().is_empty().then_some(limit)
}

pub fn button_press_ticks(button: Block) -> u8 {
    match button {
        Block::StoneButton(_) | Block::PolishedBlackstoneButton(_) => 10,
        _ => 15,
    }
}

fn set_block_audibly(world: &mut impl RedstoneWorld, pos: BlockPos, state: Block, notify: Notify) {
    let old = world.block(pos);
    update::set_block(world, pos, state, notify);
    if let Some(sound) = Sound::of_change(old, state) {
        world.play_sound(pos, sound);
    }
}
