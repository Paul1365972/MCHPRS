pub mod mask;
pub mod pattern;

use crate::player::PlayerPos;
use crate::plot::world::PlotWorld;
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_network::chunk::encode_chunk;
use mchprs_world::storage::{PalettedBitBuffer, BLOCK_STATE_DIRECT_THRESHOLD};

const EYE_HEIGHT: f64 = 1.65;

#[derive(Clone, Debug)]
pub struct RegionSnapshot {
    region: Region,
    block_states: PalettedBitBuffer,
    block_entities: Vec<(BlockPos, BlockEntity)>,
}

impl RegionSnapshot {
    pub fn take(world: &PlotWorld, region: Region) -> RegionSnapshot {
        let mut block_states =
            PalettedBitBuffer::new(region.volume(), BLOCK_STATE_DIRECT_THRESHOLD);
        let mut block_entities = Vec::new();
        for (index, pos) in region.positions().enumerate() {
            block_states.set_entry(index, world.block(pos).get_id());
            if let Some(block_entity) = world.block_entity(pos) {
                block_entities.push((pos, block_entity.clone()));
            }
        }
        RegionSnapshot {
            region,
            block_states,
            block_entities,
        }
    }

    pub fn region(&self) -> Region {
        self.region
    }

    pub fn restore(&self, world: &mut PlotWorld) {
        for (index, pos) in self.region.positions().enumerate() {
            let block = Block::from_id(self.block_states.get_entry(index))
                .expect("a snapshot holds known block states");
            world.set_block(pos, block);
            world.set_block_entity(pos, None);
        }
        set_block_entities(world, self.block_entities.iter().cloned());
    }
}

#[derive(Clone, Debug)]
pub struct WorldEditUndo {
    pub snapshots: Vec<RegionSnapshot>,
    pub plot_x: i32,
    pub plot_z: i32,
}

pub fn ray_trace_block(
    world: &impl BlockView,
    mut pos: PlayerPos,
    start_pitch: f64,
    start_yaw: f64,
    max_distance: f64,
) -> Option<BlockPos> {
    let check_distance = 0.2;
    pos.y += EYE_HEIGHT;
    let rot_x = (start_yaw + 90.0) % 360.0;
    let rot_y = -start_pitch;
    let h = check_distance * rot_y.to_radians().cos();

    let offset_x = h * rot_x.to_radians().cos();
    let offset_y = check_distance * rot_y.to_radians().sin();
    let offset_z = h * rot_x.to_radians().sin();

    let mut current_distance = 0.0;

    while current_distance < max_distance {
        let block_pos = pos.block_pos();
        let block = world.block(block_pos);

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

pub fn create_clipboard(
    world: &PlotWorld,
    origin: BlockPos,
    first_pos: BlockPos,
    second_pos: BlockPos,
) -> BlockVolume {
    BlockVolume::copy(world, Region::new(first_pos, second_pos))
        .translated(BlockPos::zero() - origin)
}

pub fn paste_clipboard(
    world: &mut PlotWorld,
    clipboard: &BlockVolume,
    at: BlockPos,
    ignore_air: bool,
) {
    for pos in clipboard.region().positions() {
        let block = clipboard.block(pos);
        if !(ignore_air && block == Block::Air) {
            let destination = pos + at;
            world.set_block(destination, block);
            if clipboard.block_entity(pos).is_none() && world.block_entity(destination).is_some() {
                world.set_block_entity(destination, None);
            }
        }
    }

    let block_entities = clipboard.block_entities();
    set_block_entities(
        world,
        block_entities.map(|(pos, entity)| (pos + at, entity.clone())),
    );
}

fn set_block_entities(
    world: &mut PlotWorld,
    block_entities: impl IntoIterator<Item = (BlockPos, BlockEntity)>,
) {
    // The client ignores the data of a block entity whose block it has not received.
    world.flush_block_changes();
    for (pos, block_entity) in block_entities {
        world.set_block_entity(pos, Some(block_entity));
    }
}

pub fn clear_area(plot: &mut PlotWorld, first_pos: BlockPos, second_pos: BlockPos) {
    for pos in plot.occupied_positions(Region::new(first_pos, second_pos)) {
        replace_block(plot, pos, Block::Air);
    }

    let start_pos = first_pos.min(second_pos);
    let end_pos = first_pos.max(second_pos);
    for chunk_x in (start_pos.x >> 4)..=(end_pos.x >> 4) {
        for chunk_z in (start_pos.z >> 4)..=(end_pos.z >> 4) {
            if let Some(chunk) = plot.chunk(chunk_x, chunk_z) {
                let chunk_data = encode_chunk(chunk);
                for packet_sender in plot.packet_senders.values() {
                    packet_sender.send_packet(&chunk_data);
                }
            }
        }
    }
}

fn replace_block(world: &mut impl BlockStore, pos: BlockPos, block: Block) -> bool {
    let changed = world.set_block(pos, block);
    let had_entity = (changed || block.has_block_entity()) && world.block_entity(pos).is_some();
    if had_entity {
        world.set_block_entity(pos, None);
    }
    changed || had_entity
}

pub fn calculate_selection_volume(first_pos: BlockPos, second_pos: BlockPos) -> i32 {
    Region::new(first_pos, second_pos).volume() as i32
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
