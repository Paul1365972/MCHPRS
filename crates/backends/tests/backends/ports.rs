use crate::common::*;
use mchprs_backends::Setup;
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::blocks::{Block, Button, Instrument, LeverFace, PressurePlate};
use mchprs_blocks::{BlockDirection, BlockPos};
use mchprs_redstone::input::{Input, InputError};
use mchprs_redstone::interpreter::Interpreter;
use mchprs_simulation::backend::{CompileError, PortError, Source};
use mchprs_simulation::input::DriveError;
use mchprs_simulation::layout::Component;
use mchprs_simulation::ports::Role;
use mchprs_simulation::simulated_world::SimulatedWorld;
use mchprs_simulation::ticks::Stop;

fn simulate(
    world: &Interpreter<BlockVolume>,
    setup: &Setup,
    declared: &[(BlockPos, Role)],
) -> Result<SimulatedWorld, CompileError> {
    let source = Source {
        declared,
        ..Source::new(world, world.blocks().region())
    };
    setup.simulate(&source)
}

fn lever_to_lamp() -> Interpreter<BlockVolume> {
    let mut world = test_world(4, 4, 4);
    make_lever(&mut world, pos(0, 1, 0));
    make_repeater(&mut world, pos(1, 1, 0), 1, BlockDirection::West);
    world.set_block(pos(2, 1, 0), Block::RedstoneLamp { lit: false });
    world
}

test_all_backends!(ports_are_in_the_layout, optimized_ports_are_in_the_layout);
fn ports_are_in_the_layout(setup: Setup) {
    check_layout(setup, true);
}

#[cfg(feature = "direct")]
fn optimized_ports_are_in_the_layout(setup: Setup) {
    check_layout(setup, false);
}

fn check_layout(setup: Setup, keeps_wire: bool) {
    let mut world = lever_to_lamp();
    make_wire(&mut world, pos(3, 1, 0));
    let plate = Block::StonePressurePlate(PressurePlate { powered: false });
    place_on_block(&mut world, pos(0, 1, 2), plate);
    world.set_block(pos(1, 0, 2), trapdoor());
    let button = Block::StoneButton(Button {
        face: LeverFace::Floor,
        facing: BlockDirection::West,
        powered: false,
    });
    place_on_block(&mut world, pos(0, 1, 3), button);
    let note_block = Block::NoteBlock {
        instrument: Instrument::Harp,
        note: 0,
        powered: false,
    };
    world.set_block(pos(1, 0, 3), note_block);
    let simulated_world = simulate(&world, &setup, &[]).expect("the world simulates");
    let layout = simulated_world.layout();
    let mut components: Vec<_> = layout
        .positions()
        .map(|pos| (pos, layout.at(pos)))
        .collect();
    let mut expected = vec![
        (pos(0, 1, 0), Some(Role::Input)),
        (pos(1, 1, 0), None),
        (pos(2, 1, 0), Some(Role::Output)),
        (pos(0, 1, 2), Some(Role::Input)),
        (pos(1, 0, 2), Some(Role::Output)),
        (pos(0, 1, 3), Some(Role::Input)),
        (pos(1, 0, 3), Some(Role::Output)),
    ];
    if keeps_wire {
        expected.push((pos(3, 1, 0), None));
    }
    let mut expected: Vec<_> = expected
        .into_iter()
        .map(|(pos, port)| {
            (
                pos,
                Some(Component {
                    port,
                    observed: true,
                }),
            )
        })
        .collect();
    components.sort_unstable_by_key(|&(pos, _)| pos);
    expected.sort_unstable_by_key(|&(pos, _)| pos);
    assert_eq!(components, expected, "with `{setup}`");
}

test_all_backends!(declared_output_is_in_the_layout);
fn declared_output_is_in_the_layout(setup: Setup) {
    let world = lever_to_lamp();
    let repeater = pos(1, 1, 0);
    let simulated_world = simulate(&world, &setup, &[(repeater, Role::Output)]);
    let component = simulated_world
        .expect("the world simulates")
        .layout()
        .at(repeater);
    assert_eq!(
        component.and_then(|component| component.port),
        Some(Role::Output),
        "with `{setup}`"
    );
}

test_all_backends!(lever_declared_as_output_refuses_drive);
fn lever_declared_as_output_refuses_drive(setup: Setup) {
    let world = lever_to_lamp();
    let lever = pos(0, 1, 0);
    let simulated_world = simulate(&world, &setup, &[(lever, Role::Output)]);
    let refused = simulated_world
        .expect("the world simulates")
        .drive(lever, Input::Toggle);
    assert!(
        matches!(refused, Err(DriveError::Refused(InputError::NotAnInput))),
        "with `{setup}`"
    );
}

test_all_backends!(declared_port_without_component_is_refused);
fn declared_port_without_component_is_refused(setup: Setup) {
    let world = lever_to_lamp();
    let empty = pos(3, 3, 3);
    let refused = simulate(&world, &setup, &[(empty, Role::Output)]);
    assert!(
        matches!(refused, Err(CompileError::Port(PortError::NoComponent(at))) if at == empty),
        "with `{setup}`"
    );
}

test_all_backends!(a_settled_world_shows_the_state_it_settled_in);
fn a_settled_world_shows_the_state_it_settled_in(setup: Setup) {
    let world = lever_to_lamp();
    let mut simulated_world = simulate(&world, &setup, &[]).expect("the world simulates");
    let lever = pos(0, 1, 0);
    let lamp = pos(2, 1, 0);
    simulated_world
        .drive(lever, Input::Toggle)
        .expect("the lever is an input");
    let ran = simulated_world.settle(10).expect("the simulation settles");
    assert_eq!(ran.stop, Some(Stop::Settled), "with `{setup}`");
    let lit = Block::RedstoneLamp { lit: true };
    assert_eq!(simulated_world.block(lamp), lit, "with `{setup}`");
}

#[cfg(feature = "vanilla")]
#[test]
fn vanilla_refuses_declared_input_it_cannot_drive() {
    let world = lever_to_lamp();
    let repeater = pos(1, 1, 0);
    let refused = simulate(&world, &vanilla(), &[(repeater, Role::Input)]);
    assert!(matches!(
        refused,
        Err(CompileError::Port(PortError::NotDrivable(at))) if at == repeater
    ));
}
