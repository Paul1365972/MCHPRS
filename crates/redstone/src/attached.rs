use crate::tick_queue::TickPriority;
use crate::update::{self, Notify};
use crate::{set_block_audibly, RedstoneWorld};
use mchprs_blocks::blocks::{Block, LeverFace};
use mchprs_blocks::{BlockDirection, BlockFace, BlockPos};

pub const BUTTON_PRIORITY: TickPriority = TickPriority::Normal;
const STONE_BUTTON_PRESS_TICKS: u8 = 10;
const WOODEN_BUTTON_PRESS_TICKS: u8 = 15;
pub const BUTTON_PRESS_TICKS: [u8; 2] = [STONE_BUTTON_PRESS_TICKS, WOODEN_BUTTON_PRESS_TICKS];

pub(crate) fn attachment(state: Block) -> Option<(LeverFace, BlockDirection, bool)> {
    match state {
        Block::Lever {
            face,
            facing,
            powered,
        } => Some((face, facing, powered)),
        _ => state
            .button()
            .map(|button| (button.face, button.facing, button.powered)),
    }
}

pub(crate) fn connected_direction(face: LeverFace, facing: BlockDirection) -> BlockFace {
    match face {
        LeverFace::Ceiling => BlockFace::Bottom,
        LeverFace::Floor => BlockFace::Top,
        LeverFace::Wall => facing.block_face(),
    }
}

pub(crate) fn update_neighbors(
    world: &mut impl RedstoneWorld,
    pos: BlockPos,
    face: LeverFace,
    facing: BlockDirection,
) {
    update::update_neighbors_at(world, pos);
    update::update_neighbors_at(
        world,
        pos.offset(connected_direction(face, facing).opposite()),
    );
}

pub(crate) fn pull_lever(
    world: &mut impl RedstoneWorld,
    pos: BlockPos,
    face: LeverFace,
    facing: BlockDirection,
    powered: bool,
) {
    let pulled = Block::Lever {
        face,
        facing,
        powered: !powered,
    };
    set_block_audibly(world, pos, pulled, Notify::All);
    update_neighbors(world, pos, face, facing);
}

pub fn button_press_ticks(button: Block) -> u8 {
    if is_stone_button(button) {
        STONE_BUTTON_PRESS_TICKS
    } else {
        WOODEN_BUTTON_PRESS_TICKS
    }
}

pub(crate) fn is_stone_button(button: Block) -> bool {
    matches!(
        button,
        Block::StoneButton(_) | Block::PolishedBlackstoneButton(_)
    )
}

pub(crate) fn press_button(world: &mut impl RedstoneWorld, pos: BlockPos, button: Block) {
    set_button_powered(world, pos, button, true);
    schedule_release(world, pos, button);
}

pub(crate) fn schedule_release(world: &mut impl RedstoneWorld, pos: BlockPos, button: Block) {
    let press_ticks = button_press_ticks(button).into();
    world.schedule_tick(pos, press_ticks, BUTTON_PRIORITY, button);
}

pub(crate) fn set_button_powered(
    world: &mut impl RedstoneWorld,
    pos: BlockPos,
    mut button: Block,
    powered: bool,
) {
    let Some(props) = button.button_mut() else {
        return;
    };
    props.powered = powered;
    let (face, facing) = (props.face, props.facing);
    set_block_audibly(world, pos, button, Notify::All);
    update_neighbors(world, pos, face, facing);
}
