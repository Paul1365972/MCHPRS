use crate::tick_queue::TickPriority;
use crate::update::{self, Notify};
use crate::{button_press_ticks, RedstoneWorld};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::{Block, LeverFace};
use mchprs_blocks::{BlockDirection, BlockFace, BlockPos};

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

pub(crate) fn can_survive(
    world: &impl BlockView,
    pos: BlockPos,
    face: LeverFace,
    facing: BlockDirection,
) -> bool {
    let toward_support = connected_direction(face, facing).opposite();
    world
        .block(pos.offset(toward_support))
        .is_face_sturdy(toward_support.opposite())
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
    update::set_block(world, pos, pulled, Notify::All);
    update_neighbors(world, pos, face, facing);
}

pub(crate) fn press_button(world: &mut impl RedstoneWorld, pos: BlockPos, button: Block) {
    set_button_powered(world, pos, button, true);
    let press_ticks = button_press_ticks(button).into();
    world.schedule_tick(pos, press_ticks, TickPriority::Normal, button);
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
    update::set_block(world, pos, button, Notify::All);
    update_neighbors(world, pos, face, facing);
}
