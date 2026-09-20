use mchprs_blocks::blocks::{Block, Comparator, ComparatorMode, LeverFace, Repeater};
use mchprs_blocks::{BlockDirection, BlockPos};
use mchprs_redpiler::backend::direct::DirectEngine;
use mchprs_redpiler::{BlockMap, CompileProgress, CompilerOptions, Engine, Input};
use mchprs_redstone::wire::make_cross;
use mchprs_world::testing::TestWorld;
use mchprs_world::World;
use std::time::{Duration, Instant};

pub fn compile(world: &TestWorld) -> (BlockMap, Box<DirectEngine>) {
    let options = CompilerOptions::default();
    let max_x = world.x_size * 16 - 1;
    let max_y = world.y_size * 16 - 1;
    let max_z = world.z_size * 16 - 1;
    let bounds = (BlockPos::new(0, 0, 0), BlockPos::new(max_x, max_y, max_z));
    let progress = CompileProgress::default();
    let netlist = mchprs_redpiler::compile(world, bounds, &options, &progress);
    let block_map = BlockMap::new(&netlist);
    let pending_ticks = block_map.map_ticks(&world.to_be_ticked);
    let engine = DirectEngine::new(&netlist, options.io_only, pending_ticks);
    (block_map, Box::new(engine))
}

#[derive(Copy, Clone)]
pub enum TestBackend {
    Redstone,
    Direct,
}

enum Runner {
    Redstone,
    Direct {
        block_map: BlockMap,
        engine: Box<DirectEngine>,
    },
}

pub struct BackendRunner {
    world: TestWorld,
    runner: Runner,
    description: &'static str,
}

impl BackendRunner {
    pub fn new(world: TestWorld, backend: TestBackend) -> BackendRunner {
        let (runner, description) = match backend {
            TestBackend::Redstone => (Runner::Redstone, "the base redstone implementation"),
            TestBackend::Direct => {
                let (block_map, engine) = compile(&world);
                (Runner::Direct { block_map, engine }, "the direct backend")
            }
        };
        BackendRunner {
            world,
            runner,
            description,
        }
    }

    pub fn tick(&mut self) {
        match &mut self.runner {
            Runner::Direct { block_map, engine } => {
                engine.run_ticks(1, Instant::now() + Duration::from_secs(10));
                block_map
                    .translate(&engine.take_changes())
                    .apply(&mut self.world);
            }
            Runner::Redstone => {
                self.world
                    .to_be_ticked
                    .sort_by_key(|e| (e.ticks_left, e.tick_priority));
                for pending in &mut self.world.to_be_ticked {
                    pending.ticks_left = pending.ticks_left.saturating_sub(1);
                }
                while self.world.to_be_ticked.first().map_or(1, |e| e.ticks_left) == 0 {
                    let entry = self.world.to_be_ticked.remove(0);
                    mchprs_redstone::tick(
                        self.world.get_block(entry.pos),
                        &mut self.world,
                        entry.pos,
                    );
                }
            }
        }
    }

    pub fn use_block(&mut self, pos: BlockPos) {
        match &mut self.runner {
            Runner::Direct { block_map, engine } => {
                let node = block_map.node_at(pos).expect("no node at pos");
                engine.input(node, Input::Interact);
                block_map
                    .translate(&engine.take_changes())
                    .apply(&mut self.world);
            }
            Runner::Redstone => {
                mchprs_redstone::on_use(self.world.get_block(pos), &mut self.world, pos);
            }
        }
    }

    pub fn check_block_powered(&self, pos: BlockPos, powered: bool) {
        assert_eq!(
            is_block_powered(self.world.get_block(pos)),
            Some(powered),
            "when testing with {}",
            self.description
        );
    }

    pub fn check_powered_for(&mut self, pos: BlockPos, powered: bool, ticks: usize) {
        for _ in 0..ticks {
            self.check_block_powered(pos, powered);
            self.tick();
        }
    }
}

fn is_block_powered(block: Block) -> Option<bool> {
    if let Some(powered) = block.clone().get_pressure_plate_powered() {
        return Some(*powered);
    }
    Some(match block {
        Block::Comparator(comparator) => comparator.powered,
        Block::RedstoneTorch { lit } => lit,
        Block::RedstoneWallTorch { lit, .. } => lit,
        Block::Repeater(repeater) => repeater.powered,
        Block::Lever { powered, .. } => powered,
        Block::StoneButton { powered, .. } => powered,
        Block::RedstoneLamp { lit } => lit,
        Block::IronTrapdoor { powered, .. } => powered,
        Block::NoteBlock { powered, .. } => powered,
        _ => return None,
    })
}

macro_rules! test_all_backends {
    ($name:ident) => {
        paste::paste! {
            #[test]
            fn [< $name _redstone >]() { $name(TestBackend::Redstone) }
            #[test]
            fn [< $name _rp_direct >]() { $name(TestBackend::Direct) }
        }
    };
}
pub(crate) use test_all_backends;

/// Helper function to create a BlockPos
pub fn pos(x: i32, y: i32, z: i32) -> BlockPos {
    BlockPos::new(x, y, z)
}

/// Place a block with a block of sandstone below it
pub fn place_on_block(world: &mut TestWorld, block_pos: BlockPos, block: Block) {
    world.set_block(block_pos - pos(0, 1, 0), Block::Sandstone {});
    world.set_block(block_pos, block);
}

pub fn trapdoor() -> Block {
    Block::IronTrapdoor {
        facing: Default::default(),
        half: Default::default(),
        powered: false,
        open: false,
        waterlogged: false,
    }
}

/// Creates a lever at `lever_pos` with a block of sandstone below it
pub fn make_lever(world: &mut TestWorld, lever_pos: BlockPos) {
    place_on_block(
        world,
        lever_pos,
        Block::Lever {
            face: LeverFace::Floor,
            facing: BlockDirection::West,
            powered: false,
        },
    );
}

/// Creates a repeater at `repeater_pos` with a block of sandstone below it
pub fn make_repeater(
    world: &mut TestWorld,
    repeater_pos: BlockPos,
    delay: u8,
    direction: BlockDirection,
) {
    place_on_block(
        world,
        repeater_pos,
        Block::Repeater(Repeater {
            delay,
            facing: direction,
            ..Default::default()
        }),
    );
}

/// Creates a wire at `wire_pos` with a block of sandstone below it
pub fn make_wire(world: &mut TestWorld, wire_pos: BlockPos) {
    place_on_block(world, wire_pos, Block::RedstoneWire(make_cross(0)));
}

/// Creates a comparator at `comp_pos` with a block of sandstone below it
pub fn make_comparator(
    world: &mut TestWorld,
    comp_pos: BlockPos,
    mode: ComparatorMode,
    facing: BlockDirection,
) {
    place_on_block(
        world,
        comp_pos,
        Block::Comparator(Comparator {
            mode,
            facing,
            ..Default::default()
        }),
    );
}
