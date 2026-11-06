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
mod plate;
mod repeater;
mod signal;
pub mod tick_queue;
mod torch;
mod update;
pub mod wire;

use mchprs_blocks::block_store::BlockStore;
use mchprs_blocks::blocks::{Block, Instrument};
use mchprs_blocks::{BlockDirection, BlockFace, BlockPos};
use tick_queue::{TickPriority, TickQueue};

pub use behavior::{can_survive, place_block, state_for_placement, use_block};
pub use diode::{diode_should_be_prioritized, is_diode};
pub use plate::set_pressure_plate;
pub use signal::{provides_strong_power, provides_weak_power};
pub use update::{neighbor_changed_at, set_block, update_neighbors_for_output_signal, Notify};

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

    fn play_note(&mut self, _pos: BlockPos, _instrument: Instrument, _note: u8) {}

    fn schedule_tick(&mut self, pos: BlockPos, delay: u32, priority: TickPriority, block: Block) {
        self.redstone().ticks.schedule(pos, delay, priority, block);
    }
}

pub fn bool_to_ss(b: bool) -> u8 {
    match b {
        true => 15,
        false => 0,
    }
}

pub fn button_press_ticks(button: Block) -> u8 {
    match button {
        Block::StoneButton(_) | Block::PolishedBlackstoneButton(_) => 10,
        _ => 15,
    }
}
