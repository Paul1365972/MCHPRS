pub mod pattern;

use crate::player::PlayerPos;
use crate::plot::PlotWorld;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::BlockPos;
use mchprs_schematic::WorldEditClipboard;
use mchprs_world::{for_each_block_mut_optimized, World};

#[derive(Clone, Debug)]
pub struct WorldEditUndo {
    pub clipboards: Vec<WorldEditClipboard>,
    pub pos: BlockPos,
    pub plot_x: i32,
    pub plot_z: i32,
}

pub fn ray_trace_block(
    world: &impl World,
    mut pos: PlayerPos,
    start_pitch: f64,
    start_yaw: f64,
    max_distance: f64,
) -> Option<BlockPos> {
    let check_distance = 0.2;

    // Player view height
    pos.y += 1.65;
    let rot_x = (start_yaw + 90.0) % 360.0;
    let rot_y = -start_pitch;
    let h = check_distance * rot_y.to_radians().cos();

    let offset_x = h * rot_x.to_radians().cos();
    let offset_y = check_distance * rot_y.to_radians().sin();
    let offset_z = h * rot_x.to_radians().sin();

    let mut current_distance = 0.0;

    while current_distance < max_distance {
        let block_pos = pos.block_pos();
        let block = world.get_block(block_pos);

        if !matches!(block, Block::Air) {
            return Some(block_pos);
        }

        pos.x += offset_x;
        pos.y += offset_y;
        pos.z += offset_z;
        current_distance += check_distance;
    }

    None
}

pub fn clear_area(plot: &mut PlotWorld, first_pos: BlockPos, second_pos: BlockPos) {
    for_each_block_mut_optimized(plot, first_pos, second_pos, |world, pos| {
        world.set_block_raw(pos, 0);
    });

    let start_pos = first_pos.min(second_pos);
    let end_pos = first_pos.max(second_pos);
    // Send modified chunks
    for chunk_x in (start_pos.x >> 4)..=(end_pos.x >> 4) {
        for chunk_z in (start_pos.z >> 4)..=(end_pos.z >> 4) {
            if let Some(chunk) = plot.get_chunk(chunk_x, chunk_z) {
                let chunk_data = chunk.encode_packet();
                for packet_sender in plot.packet_senders.values() {
                    packet_sender.send_packet(&chunk_data);
                }
            }
        }
    }
}

pub fn region_positions(
    first_pos: BlockPos,
    second_pos: BlockPos,
) -> impl Iterator<Item = BlockPos> {
    let start = first_pos.min(second_pos);
    let end = first_pos.max(second_pos);
    (start.x..=end.x).flat_map(move |x| {
        (start.y..=end.y).flat_map(move |y| (start.z..=end.z).map(move |z| BlockPos::new(x, y, z)))
    })
}

pub fn calculate_selection_volume(first_pos: BlockPos, second_pos: BlockPos) -> i32 {
    let min = first_pos.min(second_pos);
    let max = first_pos.max(second_pos);
    (max.x - min.x + 1) * (max.y - min.y + 1) * (max.z - min.z + 1)
}

pub fn calculate_expanded_selection(
    first: BlockPos,
    second: BlockPos,
    amount: BlockPos,
    contract: bool,
) -> (BlockPos, BlockPos) {
    let mut new_first = first;
    let mut new_second = second;

    for (first_coord, second_coord, delta) in [
        (&mut new_first.x, &mut new_second.x, amount.x),
        (&mut new_first.y, &mut new_second.y, amount.y),
        (&mut new_first.z, &mut new_second.z, amount.z),
    ] {
        if delta == 0 {
            continue;
        }
        let first_extends = if delta > 0 {
            *first_coord > *second_coord
        } else {
            *first_coord < *second_coord
        };
        let (moving, fixed) = if first_extends ^ contract {
            (first_coord, *second_coord)
        } else {
            (second_coord, *first_coord)
        };
        *moving += delta;
        if contract {
            *moving = if delta > 0 {
                (*moving).min(fixed)
            } else {
                (*moving).max(fixed)
            };
        }
    }

    (new_first, new_second)
}

pub fn update(plot: &mut PlotWorld, first_pos: BlockPos, second_pos: BlockPos) {
    for_each_block_mut_optimized(plot, first_pos, second_pos, |plot, pos| {
        let block = plot.get_block(pos);
        mchprs_redstone::update(block, plot, pos);
    });
}
