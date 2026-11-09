use crate::tick_queue::TickPriority;
use crate::update::{self, Notify};
use crate::{signal, RedstoneWorld};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::{BlockFace, BlockPos};

pub const DELAY_TICKS: u32 = 1;
pub const PRIORITY: TickPriority = TickPriority::Normal;

pub(crate) fn support_face(state: Block) -> BlockFace {
    match state {
        Block::RedstoneWallTorch { facing, .. } => facing.opposite().block_face(),
        _ => BlockFace::Bottom,
    }
}

pub(crate) fn neighbor_changed(world: &mut impl RedstoneWorld, pos: BlockPos, state: Block) {
    if is_lit(state) == has_neighbor_signal(world, pos, state)
        && !world.redstone().ticks.is_pending(pos, state)
    {
        world.schedule_tick(pos, DELAY_TICKS, PRIORITY, state);
    }
}

pub(crate) fn tick(world: &mut impl RedstoneWorld, pos: BlockPos, state: Block) {
    let powered = has_neighbor_signal(world, pos, state);
    if is_lit(state) == powered {
        update::set_block(world, pos, with_lit(state, !powered), Notify::All);
    }
}

fn with_lit(state: Block, lit: bool) -> Block {
    match state {
        Block::RedstoneTorch { .. } => Block::RedstoneTorch { lit },
        Block::RedstoneWallTorch { facing, .. } => Block::RedstoneWallTorch { facing, lit },
        _ => unreachable!("not a torch: {state:?}"),
    }
}

fn is_lit(state: Block) -> bool {
    matches!(
        state,
        Block::RedstoneTorch { lit: true } | Block::RedstoneWallTorch { lit: true, .. }
    )
}

fn has_neighbor_signal(world: &impl BlockView, pos: BlockPos, state: Block) -> bool {
    let toward = support_face(state);
    signal::has_signal(world, pos.offset(toward), toward)
}
