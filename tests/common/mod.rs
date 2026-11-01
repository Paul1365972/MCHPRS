use mchprs_blocks::blocks::{Block, Comparator, ComparatorMode, LeverFace, Repeater};
use mchprs_blocks::{BlockDirection, BlockPos};
use mchprs_redpiler::backend::direct::DirectEngine;
use mchprs_redpiler::{BlockMap, CompileProgress, CompilerOptions, Engine, Input};
use mchprs_redpiler_fpga::Target;
use mchprs_redstone::wire::make_cross;
use mchprs_world::testing::TestWorld;
use mchprs_world::World;
use std::time::{Duration, Instant};

#[derive(Copy, Clone)]
pub enum TestBackend {
    Redstone,
    Direct { optimize: bool },
    Phase { optimize: bool },
    Verilator { optimize: bool },
}

pub fn compile(world: &TestWorld, backend: TestBackend) -> (BlockMap, Box<dyn Engine>) {
    let optimize = match backend {
        TestBackend::Redstone => unreachable!("the base redstone implementation is not compiled"),
        TestBackend::Direct { optimize }
        | TestBackend::Phase { optimize }
        | TestBackend::Verilator { optimize } => optimize,
    };
    let options = CompilerOptions {
        optimize,
        ..Default::default()
    };
    let progress = CompileProgress::default();
    let netlist = mchprs_redpiler::compile(
        world,
        world.corners(),
        &world.to_be_ticked,
        &options,
        &progress,
    );
    let block_map = BlockMap::new(&netlist);
    let pending_ticks = block_map.map_ticks(&world.to_be_ticked);
    let target = match backend {
        TestBackend::Phase { .. } => Target::Reference,
        TestBackend::Verilator { .. } => Target::Verilator,
        _ => {
            let engine = DirectEngine::new(&netlist, options.io_only, pending_ticks);
            return (block_map, Box::new(engine));
        }
    };
    let engine = mchprs_redpiler_fpga::prepare(target, &netlist, &pending_ticks, false, &progress)
        .expect("the phase backend accepts the circuit");
    (block_map, engine)
}

enum Runner {
    Redstone,
    Compiled {
        block_map: BlockMap,
        engine: Box<dyn Engine>,
    },
}

pub struct BackendRunner {
    world: TestWorld,
    runner: Runner,
    description: &'static str,
}

impl BackendRunner {
    pub fn new(mut world: TestWorld, backend: TestBackend) -> BackendRunner {
        let (first_pos, second_pos) = world.corners();
        mchprs_redstone::update_region(&mut world, first_pos, second_pos);
        let description = match backend {
            TestBackend::Redstone => "the base redstone implementation",
            TestBackend::Direct { optimize: false } => "the direct backend",
            TestBackend::Direct { optimize: true } => "the optimized direct backend",
            TestBackend::Phase { optimize: false } => "the phase reference",
            TestBackend::Phase { optimize: true } => "the optimized phase reference",
            TestBackend::Verilator { optimize: false } => "the Verilator circuit",
            TestBackend::Verilator { optimize: true } => "the optimized Verilator circuit",
        };
        let runner = match backend {
            TestBackend::Redstone => Runner::Redstone,
            _ => {
                let (block_map, engine) = compile(&world, backend);
                Runner::Compiled { block_map, engine }
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
            Runner::Compiled { block_map, engine } => {
                engine
                    .run_ticks(1, Instant::now() + Duration::from_secs(10))
                    .unwrap();
                block_map
                    .translate(&engine.take_changes().unwrap())
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
            Runner::Compiled { block_map, engine } => {
                let node = block_map.node_at(pos).expect("no node at pos");
                engine.input(node, Input::Interact).unwrap();
                block_map
                    .translate(&engine.take_changes().unwrap())
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
            fn [< $name _rp_direct >]() { $name(TestBackend::Direct { optimize: false }) }
            #[test]
            fn [< $name _rp_direct_optimized >]() { $name(TestBackend::Direct { optimize: true }) }
            #[test]
            fn [< $name _rp_phase >]() { $name(TestBackend::Phase { optimize: false }) }
            #[test]
            fn [< $name _rp_phase_optimized >]() { $name(TestBackend::Phase { optimize: true }) }
            #[test]
            #[ignore = "needs Verilator"]
            fn [< $name _rp_verilator >]() { $name(TestBackend::Verilator { optimize: false }) }
            #[test]
            #[ignore = "needs Verilator"]
            fn [< $name _rp_verilator_optimized >]() { $name(TestBackend::Verilator { optimize: true }) }
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
