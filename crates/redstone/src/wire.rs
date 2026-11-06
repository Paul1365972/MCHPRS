use crate::signal::{self, Dust};
use crate::update::{self, Notify};
use crate::{RedstoneWorld, DIRECTIONS, HORIZONTAL};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::{Block, RedstoneWire, RedstoneWireSide};
use mchprs_blocks::{BlockDirection, BlockFace, BlockPos};

pub fn make_cross(power: u8) -> RedstoneWire {
    RedstoneWire {
        north: RedstoneWireSide::Side,
        south: RedstoneWireSide::Side,
        east: RedstoneWireSide::Side,
        west: RedstoneWireSide::Side,
        power,
    }
}

pub fn is_dot(wire: RedstoneWire) -> bool {
    HORIZONTAL
        .iter()
        .all(|&direction| wire.side(direction).is_none())
}

pub fn is_cross(wire: RedstoneWire) -> bool {
    HORIZONTAL
        .iter()
        .all(|&direction| !wire.side(direction).is_none())
}

pub fn is_illegal(wire: RedstoneWire) -> bool {
    HORIZONTAL
        .iter()
        .filter(|&&direction| wire.side(direction).is_none())
        .count()
        == 3
}

pub(crate) fn state_for_placement(world: &impl BlockView, pos: BlockPos) -> RedstoneWire {
    connection_state(world, pos, make_cross(0))
}

pub fn connection_state(world: &impl BlockView, pos: BlockPos, wire: RedstoneWire) -> RedstoneWire {
    let above_free = !world.block(pos.offset(BlockFace::Top)).is_conductor();
    let mut state = RedstoneWire {
        power: wire.power,
        ..Default::default()
    };
    for direction in HORIZONTAL {
        state = state.with_side(
            direction,
            connecting_side(world, pos, direction, above_free),
        );
    }
    if is_dot(wire) && is_dot(state) {
        return state;
    }
    let connected = |direction| !state.side(direction).is_none();
    let north_south_none = !connected(BlockDirection::North) && !connected(BlockDirection::South);
    let east_west_none = !connected(BlockDirection::East) && !connected(BlockDirection::West);
    let mut result = state;
    for (direction, alone) in [
        (BlockDirection::West, north_south_none),
        (BlockDirection::East, north_south_none),
        (BlockDirection::North, east_west_none),
        (BlockDirection::South, east_west_none),
    ] {
        if alone && !connected(direction) {
            result = result.with_side(direction, RedstoneWireSide::Side);
        }
    }
    result
}

pub(crate) fn can_survive(world: &impl BlockView, pos: BlockPos) -> bool {
    can_survive_on(world.block(pos.offset(BlockFace::Bottom)))
}

pub(crate) fn update_shape(
    world: &impl BlockView,
    pos: BlockPos,
    wire: RedstoneWire,
    direction: BlockFace,
    neighbor_state: Block,
) -> Block {
    let wire = match direction {
        BlockFace::Bottom if !can_survive_on(neighbor_state) => return Block::Air,
        BlockFace::Bottom => wire,
        BlockFace::Top => connection_state(world, pos, wire),
        _ => {
            let direction = direction.unwrap_direction();
            let above_free = !world.block(pos.offset(BlockFace::Top)).is_conductor();
            let side = connecting_side(world, pos, direction, above_free);
            if side.is_none() == wire.side(direction).is_none() && !is_cross(wire) {
                wire.with_side(direction, side)
            } else {
                connection_state(
                    world,
                    pos,
                    make_cross(wire.power).with_side(direction, side),
                )
            }
        }
    };
    Block::RedstoneWire(wire)
}

pub(crate) fn update_indirect_neighbor_shapes(
    world: &mut impl RedstoneWorld,
    pos: BlockPos,
    state: Block,
) {
    let Block::RedstoneWire(wire) = state else {
        return;
    };
    for direction in HORIZONTAL {
        let side_pos = pos.offset(direction.block_face());
        if wire.side(direction).is_none() || is_wire(world.block(side_pos)) {
            continue;
        }
        for vertical in [BlockFace::Bottom, BlockFace::Top] {
            let diagonal_pos = side_pos.offset(vertical);
            if is_wire(world.block(diagonal_pos)) {
                let toward = direction.opposite().block_face();
                let neighbor_state = world.block(diagonal_pos.offset(toward));
                update::neighbor_shape_changed(world, diagonal_pos, toward, neighbor_state);
            }
        }
    }
}

pub(crate) fn neighbor_changed(world: &mut impl RedstoneWorld, pos: BlockPos, wire: RedstoneWire) {
    if can_survive(world, pos) {
        update_power_strength(world, pos, wire);
    } else {
        update::set_block(world, pos, Block::Air, Notify::All);
    }
}

pub(crate) fn on_place(
    world: &mut impl RedstoneWorld,
    pos: BlockPos,
    wire: RedstoneWire,
    old: Block,
) {
    if is_wire(old) {
        return;
    }
    update_power_strength(world, pos, wire);
    update::update_neighbors_at(world, pos.offset(BlockFace::Top));
    update::update_neighbors_at(world, pos.offset(BlockFace::Bottom));
    update_neighbors_of_neighboring_wires(world, pos);
}

pub(crate) fn on_remove(
    world: &mut impl RedstoneWorld,
    pos: BlockPos,
    wire: RedstoneWire,
    new: Block,
) {
    if is_wire(new) {
        return;
    }
    for direction in DIRECTIONS {
        update::update_neighbors_at(world, pos.offset(direction));
    }
    update_power_strength(world, pos, wire);
    update_neighbors_of_neighboring_wires(world, pos);
}

pub(crate) fn use_block(world: &mut impl RedstoneWorld, pos: BlockPos, wire: RedstoneWire) -> bool {
    if !is_cross(wire) && !is_dot(wire) {
        return false;
    }
    let toggled = if is_cross(wire) {
        RedstoneWire::default()
    } else {
        make_cross(0)
    };
    let new_wire = connection_state(
        world,
        pos,
        RedstoneWire {
            power: wire.power,
            ..toggled
        },
    );
    if new_wire == wire {
        return false;
    }
    update::set_block(world, pos, Block::RedstoneWire(new_wire), Notify::All);
    for direction in HORIZONTAL {
        update::update_neighbors_at(world, pos.offset(direction.block_face()));
    }
    true
}

pub(crate) fn emitted(
    world: &impl BlockView,
    pos: BlockPos,
    wire: RedstoneWire,
    direction: BlockFace,
) -> u8 {
    match direction {
        _ if wire.power == 0 => 0,
        BlockFace::Bottom => 0,
        BlockFace::Top => wire.power,
        _ => {
            let toward_reader = direction.opposite().unwrap_direction();
            if connection_state(world, pos, wire)
                .side(toward_reader)
                .is_none()
            {
                0
            } else {
                wire.power
            }
        }
    }
}

fn connecting_side(
    world: &impl BlockView,
    pos: BlockPos,
    direction: BlockDirection,
    above_free: bool,
) -> RedstoneWireSide {
    let neighbor_pos = pos.offset(direction.block_face());
    let neighbor = world.block(neighbor_pos);
    if above_free {
        let supports_dust =
            matches!(neighbor, Block::IronTrapdoor { .. }) || can_survive_on(neighbor);
        if supports_dust
            && should_connect_to(world.block(neighbor_pos.offset(BlockFace::Top)), None)
        {
            return if neighbor.is_face_sturdy(direction.opposite().block_face()) {
                RedstoneWireSide::Up
            } else {
                RedstoneWireSide::Side
            };
        }
    }
    if !should_connect_to(neighbor, Some(direction))
        && (neighbor.is_conductor()
            || !should_connect_to(world.block(neighbor_pos.offset(BlockFace::Bottom)), None))
    {
        RedstoneWireSide::None
    } else {
        RedstoneWireSide::Side
    }
}

fn should_connect_to(state: Block, direction: Option<BlockDirection>) -> bool {
    match state {
        Block::RedstoneWire(_) => true,
        Block::Repeater(repeater) => {
            direction == Some(repeater.facing) || direction == Some(repeater.facing.opposite())
        }
        Block::Observer { facing, .. } => {
            direction.map(BlockDirection::block_facing) == Some(facing)
        }
        _ => signal::is_signal_source(state) && direction.is_some(),
    }
}

fn can_survive_on(below: Block) -> bool {
    below.is_face_sturdy(BlockFace::Top) || matches!(below, Block::Hopper { .. })
}

fn is_wire(state: Block) -> bool {
    matches!(state, Block::RedstoneWire(_))
}

fn wire_power(state: Block) -> u8 {
    match state {
        Block::RedstoneWire(wire) => wire.power,
        _ => 0,
    }
}

fn calculate_target_strength(world: &impl BlockView, pos: BlockPos) -> u8 {
    let strong = signal::best_neighbor_signal(world, pos, Dust::Muted);
    let mut wire = 0;
    if strong < 15 {
        let above_conductor = world.block(pos.offset(BlockFace::Top)).is_conductor();
        for direction in HORIZONTAL {
            let neighbor_pos = pos.offset(direction.block_face());
            let neighbor = world.block(neighbor_pos);
            wire = wire.max(wire_power(neighbor));
            if neighbor.is_conductor() {
                if !above_conductor {
                    wire = wire.max(wire_power(world.block(neighbor_pos.offset(BlockFace::Top))));
                }
            } else {
                wire = wire.max(wire_power(
                    world.block(neighbor_pos.offset(BlockFace::Bottom)),
                ));
            }
        }
    }
    strong.max(wire.saturating_sub(1))
}

fn update_order(pos: BlockPos) -> [BlockPos; 7] {
    let bucket = |pos: &BlockPos| {
        let hash = pos
            .y
            .wrapping_add(pos.z.wrapping_mul(31))
            .wrapping_mul(31)
            .wrapping_add(pos.x);
        (hash ^ ((hash as u32) >> 16) as i32) & 15
    };
    let mut positions = [
        pos,
        pos.offset(BlockFace::Bottom),
        pos.offset(BlockFace::Top),
        pos.offset(BlockFace::North),
        pos.offset(BlockFace::South),
        pos.offset(BlockFace::West),
        pos.offset(BlockFace::East),
    ];
    positions.sort_by_key(bucket);
    positions
}

fn update_power_strength(world: &mut impl RedstoneWorld, pos: BlockPos, wire: RedstoneWire) {
    let target = calculate_target_strength(world, pos);
    if wire.power == target {
        return;
    }
    if world.block(pos) == Block::RedstoneWire(wire) {
        let powered = RedstoneWire {
            power: target,
            ..wire
        };
        update::set_block(world, pos, Block::RedstoneWire(powered), Notify::Shapes);
    }
    for neighbor_pos in update_order(pos) {
        update::update_neighbors_at(world, neighbor_pos);
    }
}

fn update_neighbors_of_neighboring_wires(world: &mut impl RedstoneWorld, pos: BlockPos) {
    for direction in HORIZONTAL {
        check_corner_change_at(world, pos.offset(direction.block_face()));
    }
    for direction in HORIZONTAL {
        let neighbor_pos = pos.offset(direction.block_face());
        if world.block(neighbor_pos).is_conductor() {
            check_corner_change_at(world, neighbor_pos.offset(BlockFace::Top));
        } else {
            check_corner_change_at(world, neighbor_pos.offset(BlockFace::Bottom));
        }
    }
}

fn check_corner_change_at(world: &mut impl RedstoneWorld, pos: BlockPos) {
    if !is_wire(world.block(pos)) {
        return;
    }
    update::update_neighbors_at(world, pos);
    for direction in DIRECTIONS {
        update::update_neighbors_at(world, pos.offset(direction));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::interpreter::Interpreter;
    use mchprs_blocks::block_store::BlockStore;
    use mchprs_blocks::block_volume::BlockVolume;
    use mchprs_blocks::blocks::LeverFace;
    use mchprs_blocks::region::Region;

    // Iteration orders of a real `java.util.HashSet`, as indexes into the insertion order
    #[test]
    fn update_order_matches_java() {
        let java_orders = [
            (BlockPos::new(0, 0, 0), [0, 3, 5, 4, 6, 1, 2]),
            (BlockPos::new(-5, 64, 13), [2, 3, 5, 0, 1, 4, 6]),
            (BlockPos::new(1000000, 100, -999999), [1, 4, 6, 0, 2, 3, 5]),
            (BlockPos::new(123, 7, -456), [2, 3, 5, 1, 4, 6, 0]),
            (
                BlockPos::new(-30000000, 319, 29999999),
                [2, 3, 5, 1, 4, 6, 0],
            ),
            (BlockPos::new(17, -64, -3), [1, 4, 6, 0, 2, 3, 5]),
        ];
        for (pos, java_order) in java_orders {
            let inserted = [
                pos,
                pos.offset(BlockFace::Bottom),
                pos.offset(BlockFace::Top),
                pos.offset(BlockFace::North),
                pos.offset(BlockFace::South),
                pos.offset(BlockFace::West),
                pos.offset(BlockFace::East),
            ];
            assert_eq!(update_order(pos), java_order.map(|idx| inserted[idx]));
        }
    }

    // Recorded on a vanilla 1.20.4 server
    #[test]
    fn cross_reshapes_when_a_lever_beside_it_turns_on() {
        let dust = BlockPos::new(-51, 20, 86);
        let lever = BlockPos::new(-51, 20, 87);
        let region = Region::new(BlockPos::new(-53, 19, 84), BlockPos::new(-47, 22, 89));
        let mut world = Interpreter::new(BlockVolume::empty(region), Vec::new());
        world.set_block(dust.offset(BlockFace::Bottom), Block::Stone);
        world.set_block(lever.offset(BlockFace::Bottom), Block::Stone);
        let cross = make_cross(0).with_side(BlockDirection::North, RedstoneWireSide::Up);
        world.set_block(dust, Block::RedstoneWire(cross));
        world.set_block(
            lever,
            Block::Lever {
                face: LeverFace::Floor,
                facing: BlockDirection::North,
                powered: false,
            },
        );

        world.use_block(lever);

        let line = RedstoneWire {
            east: RedstoneWireSide::None,
            west: RedstoneWireSide::None,
            ..make_cross(15)
        };
        assert_eq!(world.block(dust), Block::RedstoneWire(line));
    }
}
