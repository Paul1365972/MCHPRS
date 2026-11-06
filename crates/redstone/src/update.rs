use crate::{behavior, comparator, wire, RedstoneWorld, HORIZONTAL};
use mchprs_blocks::blocks::Block;
use mchprs_blocks::{BlockFace, BlockPos};

/// Vanilla's `NeighborUpdater.UPDATE_ORDER`
const UPDATE_ORDER: [BlockFace; 6] = [
    BlockFace::West,
    BlockFace::East,
    BlockFace::Bottom,
    BlockFace::Top,
    BlockFace::North,
    BlockFace::South,
];

/// Vanilla's `BlockBehaviour.UPDATE_SHAPE_ORDER`
const UPDATE_SHAPE_ORDER: [BlockFace; 6] = [
    BlockFace::West,
    BlockFace::East,
    BlockFace::North,
    BlockFace::South,
    BlockFace::Bottom,
    BlockFace::Top,
];

const MAX_CHAINED_NEIGHBOR_UPDATES: u32 = 1_000_000;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Notify {
    All,
    Shapes,
}

#[derive(Clone, Copy, Debug)]
enum Update {
    Neighbor(BlockPos),
    Shape {
        pos: BlockPos,
        direction: BlockFace,
        neighbor_state: Block,
    },
}

#[derive(Debug, Default)]
pub(crate) struct NeighborUpdater {
    stack: Vec<Update>,
    count: u32,
}

pub fn set_block(
    world: &mut impl RedstoneWorld,
    pos: BlockPos,
    state: Block,
    notify: Notify,
) -> bool {
    let old = world.block(pos);
    if !world.set_block(pos, state) {
        return false;
    }
    behavior::on_remove(world, pos, old, state);
    if old.has_block_entity() && !old.is_same_kind(state) {
        world.set_block_entity(pos, None);
    }
    if !world.block(pos).is_same_kind(state) {
        return false;
    }
    behavior::on_place(world, pos, state, old);
    if world.block(pos) != state {
        return true;
    }
    if notify == Notify::All {
        update_neighbors_at(world, pos);
        if comparator::analog_output(world, pos, state).is_some() {
            update_neighbors_for_output_signal(world, pos);
        }
    }
    wire::update_indirect_neighbor_shapes(world, pos, old);
    for direction in UPDATE_SHAPE_ORDER {
        neighbor_shape_changed(world, pos.offset(direction), direction.opposite(), state);
    }
    wire::update_indirect_neighbor_shapes(world, pos, state);
    true
}

pub(crate) fn neighbor_shape_changed(
    world: &mut impl RedstoneWorld,
    pos: BlockPos,
    direction: BlockFace,
    neighbor_state: Block,
) {
    add_and_run(
        world,
        [Update::Shape {
            pos,
            direction,
            neighbor_state,
        }],
    );
}

pub(crate) fn update_neighbors_at(world: &mut impl RedstoneWorld, pos: BlockPos) {
    update_neighbors_at_except(world, pos, None);
}

pub(crate) fn update_neighbors_at_except(
    world: &mut impl RedstoneWorld,
    pos: BlockPos,
    skip: Option<BlockFace>,
) {
    let neighbors = UPDATE_ORDER
        .into_iter()
        .filter(|&direction| Some(direction) != skip)
        .map(|direction| Update::Neighbor(pos.offset(direction)));
    add_and_run(world, neighbors);
}

pub fn neighbor_changed_at(world: &mut impl RedstoneWorld, pos: BlockPos) {
    add_and_run(world, [Update::Neighbor(pos)]);
}

pub fn update_neighbors_for_output_signal(world: &mut impl RedstoneWorld, pos: BlockPos) {
    for direction in HORIZONTAL {
        let mut neighbor_pos = pos.offset(direction.block_face());
        let mut state = world.block(neighbor_pos);
        if !matches!(state, Block::Comparator(_)) && state.is_conductor() {
            neighbor_pos = neighbor_pos.offset(direction.block_face());
            state = world.block(neighbor_pos);
        }
        if matches!(state, Block::Comparator(_)) {
            neighbor_changed_at(world, neighbor_pos);
        }
    }
}

fn add_and_run(world: &mut impl RedstoneWorld, updates: impl IntoIterator<Item = Update>) {
    let updater = &mut world.redstone().updater;
    let nested = updater.count > 0;
    if updater.count < MAX_CHAINED_NEIGHBOR_UPDATES {
        updater.stack.extend(updates);
    }
    updater.count = updater.count.saturating_add(1);
    if !nested {
        updater.stack.reverse();
        run_updates(world);
    }
}

fn run_updates(world: &mut impl RedstoneWorld) {
    while let Some(update) = world.redstone().updater.stack.pop() {
        let added_from = world.redstone().updater.stack.len();
        run(world, update);
        world.redstone().updater.stack[added_from..].reverse();
    }
    world.redstone().updater.count = 0;
}

fn run(world: &mut impl RedstoneWorld, update: Update) {
    match update {
        Update::Neighbor(pos) => {
            let state = world.block(pos);
            behavior::neighbor_changed(world, pos, state);
        }
        Update::Shape {
            pos,
            direction,
            neighbor_state,
        } => {
            let state = world.block(pos);
            let new_state = behavior::update_shape(world, pos, state, direction, neighbor_state);
            if new_state != state {
                if new_state == Block::Air {
                    set_block(world, pos, Block::Air, Notify::All);
                } else {
                    set_block(world, pos, new_state, Notify::Shapes);
                }
            }
        }
    }
}
