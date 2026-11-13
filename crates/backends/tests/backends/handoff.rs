#![cfg(feature = "direct")]

use crate::common::*;
use mchprs_backends::Setup;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::{Block, RedstoneWire, RedstoneWireSide};
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::BlockDirection;
use mchprs_redstone::input::Input;
use mchprs_redstone::interpreter::Interpreter;
use mchprs_redstone::{settle, wire as dust};
use mchprs_simulation::backend::Source;
use mchprs_simulation::readback::Readback;

#[test]
fn optimized_write_back_refreshes_wire_and_continues() {
    let mut world = test_world(5, 2, 1);
    let lever = pos(0, 1, 0);
    let wire = pos(1, 1, 0);
    let repeater = pos(2, 1, 0);
    let lamp = pos(3, 1, 0);
    make_lever(&mut world, lever);
    place_on_block(
        &mut world,
        wire,
        Block::RedstoneWire(RedstoneWire {
            north: RedstoneWireSide::None,
            south: RedstoneWireSide::None,
            ..dust::cross(SignalStrength::Zero)
        }),
    );
    make_repeater(&mut world, repeater, 3, BlockDirection::West);
    place_on_block(&mut world, lamp, Block::RedstoneLamp { lit: false });
    let region = world.blocks().region();
    settle(&mut world, region, 100).unwrap();
    let mut simulated = Setup::parse("direct -o")
        .unwrap()
        .simulate(&Source::new(&world, region))
        .unwrap();
    simulated.drive(lever, Input::Toggle).unwrap();
    world.drive(lever, Input::Toggle).unwrap();
    simulated.run(1).unwrap();
    world.tick();
    assert!(!simulated.is_current(wire));
    assert_ne!(simulated.block(wire), world.block(wire));
    assert!(matches!(world.block(repeater), Block::Repeater(r) if !r.powered));
    let (blocks, Readback::Complete(write_back)) = simulated.stop().unwrap() else {
        panic!("direct provides complete readback");
    };
    assert!(!write_back.ticks.is_empty());
    let mut resumed = Interpreter::new(blocks, Vec::new());
    write_back.apply(&mut resumed);
    for _ in 0..5 {
        for at in region.positions() {
            assert_eq!(resumed.block(at), world.block(at), "at {at}");
        }
        assert_eq!(resumed.pending_ticks(), world.pending_ticks());
        resumed.tick();
        world.tick();
    }
    assert_eq!(resumed.block(lamp), Block::RedstoneLamp { lit: true });
}
