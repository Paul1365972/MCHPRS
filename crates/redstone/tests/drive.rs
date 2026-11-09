use mchprs_blocks::block_store::BlockStore;
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::blocks::{Button, LeverFace, PressurePlate};
use mchprs_blocks::region::Region;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::BlockDirection;
use mchprs_blocks::BlockPos;
use mchprs_redstone::input::{is_drivable, Input, InputError};
use mchprs_redstone::interpreter::Interpreter;

#[test]
fn drive_takes_what_a_player_can_do() {
    let (face, facing) = (LeverFace::Floor, BlockDirection::North);
    let lever = Block::Lever {
        face,
        facing,
        powered: false,
    };
    let button = Block::StoneButton(Button {
        face,
        facing,
        powered: false,
    });
    let plate = Block::StonePressurePlate(PressurePlate { powered: false });
    let on_off = [
        Input::Toggle,
        Input::Set(SignalStrength::Zero),
        Input::Set(SignalStrength::Fifteen),
    ];
    let touch = [Input::Toggle, Input::Touch(false), Input::Touch(true)];
    let accepted: [(Block, &[Input]); 4] = [
        (lever, &on_off),
        (button, &[Input::Press]),
        (plate, &touch),
        (Block::Stone, &[]),
    ];
    let pos = BlockPos::new(0, 0, 0);
    let region = Region::new(BlockPos::new(-2, -2, -2), BlockPos::new(2, 2, 2));
    for (block, inputs) in accepted {
        assert_eq!(is_drivable(block), !inputs.is_empty(), "{block}");
        for input in Input::all() {
            let mut world = Interpreter::new(BlockVolume::empty(region), Vec::new());
            world.set_block(pos, block);
            let expected = if inputs.contains(&input) {
                Ok(())
            } else if inputs.is_empty() {
                Err(InputError::NotAnInput)
            } else {
                Err(InputError::Unsupported(input))
            };
            assert_eq!(world.drive(pos, input), expected, "{block} {input:?}");
        }
    }
}
