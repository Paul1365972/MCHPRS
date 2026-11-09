use crate::common::*;
use mchprs_backends::Setup;
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::blocks::{Block, PressurePlate};
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::BlockDirection;
use mchprs_redstone::input::Input;
use mchprs_redstone::tick_queue::TickPriority;
use mchprs_redstone::wire;
use mchprs_redstone::RedstoneWorld;

test_all_backends!(lever_on_off);
fn lever_on_off(setup: Setup) {
    let lever_pos = pos(0, 1, 0);

    let mut world = test_world(16, 16, 16);
    make_lever(&mut world, lever_pos);

    let mut runner = Runner::new(&world, &setup);
    runner.check_block_powered(lever_pos, false);

    runner.use_block(lever_pos);
    runner.check_block_powered(lever_pos, true);

    runner.use_block(lever_pos);
    runner.check_block_powered(lever_pos, false);
}

test_all_backends!(trapdoor_on_off);
fn trapdoor_on_off(setup: Setup) {
    let lever_pos = pos(0, 1, 0);
    let trapdoor_pos = pos(1, 0, 0);

    let mut world = test_world(16, 16, 16);
    make_lever(&mut world, lever_pos);
    world.set_block(trapdoor_pos, trapdoor());

    let mut runner = Runner::new(&world, &setup);
    runner.check_block_powered(trapdoor_pos, false);

    runner.use_block(lever_pos);
    runner.check_block_powered(trapdoor_pos, true);

    runner.use_block(lever_pos);
    runner.check_block_powered(trapdoor_pos, false);
}

test_all_backends!(pressure_plate_on_off);
fn pressure_plate_on_off(setup: Setup) {
    let plate_pos = pos(0, 1, 0);
    let trapdoor_pos = pos(1, 0, 0);

    let mut world = test_world(16, 16, 16);
    place_on_block(
        &mut world,
        plate_pos,
        Block::StonePressurePlate(PressurePlate { powered: false }),
    );
    world.set_block(trapdoor_pos, trapdoor());

    let mut runner = Runner::new(&world, &setup);
    runner.check_block_powered(trapdoor_pos, false);

    runner.drive(plate_pos, Input::Touch(true));
    runner.check_block_powered(trapdoor_pos, true);

    runner.drive(plate_pos, Input::Touch(false));
    for _ in 0..10 {
        runner.tick();
    }
    runner.check_block_powered(trapdoor_pos, false);
}

test_all_backends!(lamp_on_off);
fn lamp_on_off(setup: Setup) {
    let lever_pos = pos(0, 1, 0);
    let lamp_pos = pos(1, 0, 0);

    let mut world = test_world(16, 16, 16);
    make_lever(&mut world, lever_pos);
    world.set_block(lamp_pos, Block::RedstoneLamp { lit: false });

    let mut runner = Runner::new(&world, &setup);
    runner.check_block_powered(lamp_pos, false);

    runner.use_block(lever_pos);
    runner.check_block_powered(lamp_pos, true);

    runner.use_block(lever_pos);
    runner.check_powered_for(lamp_pos, true, 2);
    runner.check_block_powered(lamp_pos, false);
}

test_all_backends!(wall_torch_on_off);
fn wall_torch_on_off(setup: Setup) {
    let lever_pos = pos(0, 1, 0);
    let torch_pos = pos(1, 0, 0);

    let mut world = test_world(16, 16, 16);
    make_lever(&mut world, lever_pos);
    world.set_block(
        torch_pos,
        Block::RedstoneWallTorch {
            lit: true,
            facing: BlockDirection::East,
        },
    );

    let mut runner = Runner::new(&world, &setup);
    runner.check_block_powered(torch_pos, true);

    runner.use_block(lever_pos);
    runner.check_powered_for(torch_pos, true, 1);
    runner.check_block_powered(torch_pos, false);

    runner.use_block(lever_pos);
    runner.check_powered_for(torch_pos, false, 1);
    runner.check_block_powered(torch_pos, true);
}

test_all_backends!(torch_on_off);
fn torch_on_off(setup: Setup) {
    let lever_pos = pos(0, 2, 0);
    let torch_pos = pos(2, 2, 0);

    let mut world = test_world(16, 16, 16);
    make_lever(&mut world, lever_pos);
    make_wire(&mut world, pos(1, 1, 0));
    place_on_block(&mut world, torch_pos, Block::RedstoneTorch { lit: true });

    let mut runner = Runner::new(&world, &setup);
    runner.check_block_powered(torch_pos, true);

    runner.use_block(lever_pos);
    runner.check_powered_for(torch_pos, true, 1);
    runner.check_block_powered(torch_pos, false);

    runner.use_block(lever_pos);
    runner.check_powered_for(torch_pos, false, 1);
    runner.check_block_powered(torch_pos, true);
}

test_all_backends!(repeater_on_off);
fn repeater_on_off(setup: Setup) {
    let lever_pos = pos(0, 2, 0);
    let trapdoor_pos = pos(2, 1, 0);

    for delay in 1..=4 {
        let mut world = test_world(16, 16, 16);
        make_lever(&mut world, lever_pos);
        make_repeater(&mut world, pos(1, 1, 0), delay as u8, BlockDirection::West);
        world.set_block(trapdoor_pos, trapdoor());

        let mut runner = Runner::new(&world, &setup);
        runner.check_block_powered(trapdoor_pos, false);

        runner.use_block(lever_pos);
        runner.check_powered_for(trapdoor_pos, false, delay);
        runner.check_block_powered(trapdoor_pos, true);
        runner.use_block(lever_pos);
        runner.check_powered_for(trapdoor_pos, true, delay);
        runner.check_block_powered(trapdoor_pos, false);

        runner.use_block(lever_pos);
        runner.use_block(lever_pos);
        runner.check_powered_for(trapdoor_pos, false, delay);
        runner.check_powered_for(trapdoor_pos, true, delay);
        runner.check_block_powered(trapdoor_pos, false);
    }
}

test_all_backends!(wire_barely_reaches);
fn wire_barely_reaches(setup: Setup) {
    let lever_pos = pos(0, 1, 0);
    let trapdoor_pos = pos(16, 1, 0);

    let mut world = test_world(32, 16, 16);
    make_lever(&mut world, lever_pos);
    for x in 1..=15 {
        make_wire(&mut world, pos(x, 1, 0));
    }
    world.set_block(trapdoor_pos, trapdoor());

    let mut runner = Runner::new(&world, &setup);
    runner.check_block_powered(trapdoor_pos, false);
    runner.use_block(lever_pos);
    runner.check_block_powered(trapdoor_pos, true);
    runner.use_block(lever_pos);
    runner.check_block_powered(trapdoor_pos, false);
}

test_all_backends!(wire_no_reach);
fn wire_no_reach(setup: Setup) {
    let lever_pos = pos(0, 1, 0);
    let trapdoor_pos = pos(17, 1, 0);

    let mut world = test_world(32, 16, 16);
    make_lever(&mut world, lever_pos);
    for x in 1..=16 {
        make_wire(&mut world, pos(x, 1, 0));
    }
    world.set_block(trapdoor_pos, trapdoor());

    let mut runner = Runner::new(&world, &setup);
    runner.check_block_powered(trapdoor_pos, false);
    runner.use_block(lever_pos);
    runner.check_block_powered(trapdoor_pos, false);
    runner.use_block(lever_pos);
    runner.check_block_powered(trapdoor_pos, false);
}

test_all_backends!(ground_torch_does_not_power_block_below);
/// https://github.com/MCHPR/MCHPRS/issues/218
fn ground_torch_does_not_power_block_below(setup: Setup) {
    let torch_pos = pos(0, 1, 0);
    let lamp_pos = pos(0, 0, 0);

    let mut world = test_world(16, 16, 16);
    world.set_block(lamp_pos, Block::RedstoneLamp { lit: true });
    world.set_block(torch_pos, Block::RedstoneTorch { lit: true });

    let lamp = world.block(lamp_pos);
    world.schedule_tick(lamp_pos, 1, TickPriority::Normal, lamp);

    let mut runner = Runner::new(&world, &setup);
    runner.tick();
    runner.check_block_powered(lamp_pos, false);
}

test_all_backends!(pending_repeater_pulses_without_input);
fn pending_repeater_pulses_without_input(setup: Setup) {
    let repeater_pos = pos(0, 1, 0);
    let trapdoor_pos = pos(1, 1, 0);

    let mut world = test_world(16, 16, 16);
    make_repeater(&mut world, repeater_pos, 1, BlockDirection::West);
    world.set_block(trapdoor_pos, trapdoor());
    let repeater = world.block(repeater_pos);
    world.schedule_tick(repeater_pos, 1, TickPriority::Higher, repeater);

    let mut runner = Runner::new(&world, &setup);
    runner.check_block_powered(trapdoor_pos, false);
    runner.tick();
    runner.check_block_powered(trapdoor_pos, true);
    runner.tick();
    runner.check_block_powered(trapdoor_pos, false);
}

test_all_backends!(wire_on_wire);
fn wire_on_wire(setup: Setup) {
    let lever_pos = pos(0, 1, 0);
    let wire_pos = pos(1, 1, 0);
    let trapdoor_pos = pos(2, 1, 0);

    let mut world = test_world(16, 16, 16);
    make_lever(&mut world, lever_pos);
    make_wire(&mut world, wire_pos);
    // Wire stacked straight on wire, as only a paste can place it
    for y in 2..5 {
        world.set_block(
            pos(1, y, 0),
            Block::RedstoneWire(wire::cross(SignalStrength::Zero)),
        );
    }
    world.set_block(trapdoor_pos, trapdoor());

    let mut runner = Runner::new(&world, &setup);
    runner.check_block_powered(trapdoor_pos, false);

    runner.use_block(lever_pos);
    runner.check_block_powered(trapdoor_pos, true);

    runner.use_block(lever_pos);
    runner.check_block_powered(trapdoor_pos, false);
}
