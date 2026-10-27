use mchprs_backends::Setup;
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::blocks::{Block, LeverFace, Repeater};
use mchprs_blocks::region::Region;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::{BlockDirection, BlockPos};
use mchprs_redstone::input::Input;
use mchprs_redstone::interpreter::Interpreter;
use mchprs_redstone::wire;
use mchprs_simulation::backend::Source;
use mchprs_simulation::component::powered_at;
use mchprs_simulation::simulated_world::SimulatedWorld;

macro_rules! test_all_backends {
    ($name:ident) => {
        test_all_backends!($name, $name);
    };
    ($name:ident, $optimized:ident) => {
        mod $name {
            #[cfg(feature = "vanilla")]
            #[test]
            fn vanilla() {
                super::$name(crate::common::vanilla());
            }

            #[cfg(feature = "direct")]
            #[test]
            fn direct() {
                super::$name(mchprs_backends::Setup::parse("direct").unwrap());
            }

            #[cfg(feature = "direct")]
            #[test]
            fn direct_optimized() {
                super::$optimized(mchprs_backends::Setup::parse("direct --optimize").unwrap());
            }

            #[cfg(feature = "phase")]
            #[test]
            fn phase() {
                super::$name(mchprs_backends::Setup::parse("phase").unwrap());
            }

            #[cfg(feature = "phase")]
            #[test]
            fn phase_optimized() {
                super::$optimized(mchprs_backends::Setup::parse("phase --optimize").unwrap());
            }
        }
    };
}
pub(crate) use test_all_backends;

pub struct Runner {
    setup: Setup,
    simulated_world: SimulatedWorld,
}

impl Runner {
    pub fn new(world: &Interpreter<BlockVolume>, setup: &Setup) -> Runner {
        let source = Source {
            ticks: world.tick_entries(),
            ..Source::new(world, world.blocks().region())
        };
        let simulated_world = setup.simulate(&source).expect("the test circuit simulates");
        Runner {
            setup: setup.clone(),
            simulated_world,
        }
    }

    pub fn tick(&mut self) {
        self.simulated_world.run(1).expect("the simulation runs");
    }

    pub fn use_block(&mut self, pos: BlockPos) {
        let block = self.simulated_world.block(pos);
        let input = Input::of_use(block).unwrap_or_else(|| panic!("{block:?} can't be used"));
        self.drive(pos, input);
    }

    pub fn drive(&mut self, pos: BlockPos, input: Input) {
        self.simulated_world
            .drive(pos, input)
            .expect("the driven block is an input");
    }

    pub fn check_block_powered(&self, pos: BlockPos, powered: bool) {
        assert_eq!(
            powered_at(&self.simulated_world, pos),
            Some(powered),
            "with `{}`",
            self.setup
        );
    }

    pub fn check_powered_for(&mut self, pos: BlockPos, powered: bool, ticks: usize) {
        for _ in 0..ticks {
            self.check_block_powered(pos, powered);
            self.tick();
        }
    }
}

#[cfg(feature = "vanilla")]
pub fn vanilla() -> Setup {
    Setup::new(
        &mchprs_backends::vanilla::VanillaBackend,
        Default::default(),
    )
}

pub fn test_world(size_x: i32, size_y: i32, size_z: i32) -> Interpreter<BlockVolume> {
    let region = Region::new(
        BlockPos::new(0, 0, 0),
        BlockPos::new(size_x - 1, size_y - 1, size_z - 1),
    );
    Interpreter::new(BlockVolume::empty(region), Vec::new())
}

pub fn pos(x: i32, y: i32, z: i32) -> BlockPos {
    BlockPos::new(x, y, z)
}

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

pub fn make_wire(world: &mut impl BlockStore, wire_pos: BlockPos) {
    place_on_block(
        world,
        wire_pos,
        Block::RedstoneWire(wire::cross(SignalStrength::Zero)),
    );
}
