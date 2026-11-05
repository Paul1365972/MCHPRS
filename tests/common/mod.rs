use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::blocks::{Block, LeverFace, Repeater};
use mchprs_blocks::region::Region;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::{BlockDirection, BlockPos};
use mchprs_redpiler::engine::Input;
use mchprs_redpiler::simulation::Simulation;
use mchprs_redpiler_backends::BackendSetup;
use mchprs_redstone::interpreter::Interpreter;
use mchprs_redstone::wire::make_cross;

struct RedpilerInstance {
    label: String,
    simulation: Simulation,
}

impl RedpilerInstance {
    fn new(world: &Interpreter<BlockVolume>, setup: &str) -> RedpilerInstance {
        let setup = BackendSetup::parse(setup).expect("the test setup parses");
        let simulation = setup
            .simulate(world, world.blocks().region(), world.pending_ticks())
            .expect("the test circuit simulates");
        RedpilerInstance {
            label: setup.to_string(),
            simulation,
        }
    }

    fn sync(&mut self, world: &mut Interpreter<BlockVolume>) {
        self.simulation
            .sync()
            .expect("the engine does not fail")
            .apply(world);
    }
}

#[derive(Copy, Clone)]
pub enum TestBackend {
    Redstone,
    Direct { optimize: bool },
}

pub struct BackendRunner {
    world: Interpreter<BlockVolume>,
    redpiler: Option<RedpilerInstance>,
}

impl BackendRunner {
    pub fn new(world: Interpreter<BlockVolume>, backend: TestBackend) -> BackendRunner {
        match backend {
            TestBackend::Redstone => BackendRunner {
                world,
                redpiler: None,
            },
            TestBackend::Direct { optimize } => BackendRunner {
                redpiler: Some(RedpilerInstance::new(
                    &world,
                    if optimize { "direct -o" } else { "direct" },
                )),
                world,
            },
        }
    }

    pub fn tick(&mut self) {
        if let Some(redpiler) = &mut self.redpiler {
            redpiler
                .simulation
                .run(1)
                .expect("the engine does not fail");
            redpiler.sync(&mut self.world);
            return;
        }

        self.world.tick();
    }

    pub fn use_block(&mut self, pos: BlockPos) {
        if let Some(redpiler) = &mut self.redpiler {
            let input = match self.world.block(pos) {
                Block::Lever { .. } => Input::Toggle,
                block if block.button().is_some() => Input::Press,
                block => panic!("{block:?} can't be used"),
            };
            redpiler
                .simulation
                .drive(pos, input)
                .expect("the used block is an input");
            redpiler.sync(&mut self.world);
            return;
        }
        self.world.use_block(pos);
    }

    pub fn check_block_powered(&self, pos: BlockPos, powered: bool) {
        if let Some(redpiler) = &self.redpiler {
            assert_eq!(
                is_block_powered(self.world.block(pos)),
                Some(powered),
                "when testing on redpiler with `{}`",
                redpiler.label
            );
            return;
        }
        assert_eq!(
            is_block_powered(self.world.block(pos)),
            Some(powered),
            "when testing with the base redstone implementation"
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
    if let Some(powered) = block.pressure_plate_powered() {
        return Some(powered);
    }
    Some(match block {
        Block::Comparator(comparator) => comparator.powered,
        Block::RedstoneTorch { lit } => lit,
        Block::RedstoneWallTorch { lit, .. } => lit,
        Block::Repeater(repeater) => repeater.powered,
        Block::Lever { powered, .. } => powered,
        Block::RedstoneLamp { lit } => lit,
        Block::IronTrapdoor { powered, .. } => powered,
        Block::NoteBlock { powered, .. } => powered,
        _ => return block.button().map(|button| button.powered),
    })
}

macro_rules! test_all_backends {
    ($name:ident) => {
        paste::paste! {
            #[test]
            fn [< $name _redstone >]() { $name(TestBackend::Redstone) }
            #[test]
            fn [< $name _rp_direct >]() {
                $name(TestBackend::Direct { optimize: false })
            }
            #[test]
            fn [< $name _rp_direct_optimized >]() {
                $name(TestBackend::Direct { optimize: true })
            }
        }
    };
}
pub(crate) use test_all_backends;

pub fn test_world(size_x: i32, size_y: i32, size_z: i32) -> Interpreter<BlockVolume> {
    let region = Region::new(
        BlockPos::new(0, 0, 0),
        BlockPos::new(size_x - 1, size_y - 1, size_z - 1),
    );
    Interpreter::new(BlockVolume::empty(region), Vec::new())
}

/// Helper function to create a BlockPos
pub fn pos(x: i32, y: i32, z: i32) -> BlockPos {
    BlockPos::new(x, y, z)
}

/// Place a block with a block of sandstone below it
pub fn place_on_block(world: &mut impl BlockStore, block_pos: BlockPos, block: Block) {
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
pub fn make_lever(world: &mut impl BlockStore, lever_pos: BlockPos) {
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
    world: &mut impl BlockStore,
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
pub fn make_wire(world: &mut impl BlockStore, wire_pos: BlockPos) {
    place_on_block(
        world,
        wire_pos,
        Block::RedstoneWire(make_cross(SignalStrength::Zero)),
    );
}
