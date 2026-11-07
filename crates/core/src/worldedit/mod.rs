pub mod mask;
pub mod pattern;
mod suggestions;

use crate::player::PlayerPos;
use crate::plot::PlotWorld;
use mchprs_blocks::block_state_spec::{parse_properties, BlockStateSpec};
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_network::chunk::encode_chunk;
use rustc_hash::FxHashMap;

#[derive(Clone, Debug)]
pub struct WorldEditUndo {
    pub clipboards: Vec<BlockVolume>,
    pub pos: BlockPos,
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
    let region = Region::new(first_pos, second_pos);
    let mut clipboard = BlockVolume::empty(region);
    for pos in region.positions() {
        let block = world.block(pos);
        clipboard.set_block(pos, block);
        if block.has_block_entity()
            && let Some(block_entity) = world.block_entity(pos)
        {
            clipboard.set_block_entity(pos, Some(block_entity.clone()));
        }
    }
    clipboard.translated(BlockPos::zero() - origin)
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

    // Send block changes before we send block entity data, otherwise it'll be ignored
    world.flush_block_changes();

    for (pos, block_entity) in clipboard.block_entities() {
        world.set_block_entity(pos + at, Some(block_entity.clone()));
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

pub fn clear_area(plot: &mut PlotWorld, first_pos: BlockPos, second_pos: BlockPos) {
    for pos in plot
        .occupied_regions(Region::new(first_pos, second_pos))
        .into_iter()
        .flat_map(Region::positions)
    {
        replace_block(plot, pos, Block::Air);
    }

    let start_pos = first_pos.min(second_pos);
    let end_pos = first_pos.max(second_pos);
    // Send modified chunks
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

fn split_top_level_commas(input: &str) -> Vec<&str> {
    let mut parts = Vec::new();
    let mut depth = 0i32;
    let mut start = 0;
    for (i, ch) in input.char_indices() {
        match ch {
            '[' => depth += 1,
            ']' => depth -= 1,
            ',' if depth == 0 => {
                parts.push(&input[start..i]);
                start = i + 1;
            }
            _ => {}
        }
    }
    parts.push(&input[start..]);
    parts
}

fn parse_raw_block(input: &str) -> Result<Block, String> {
    let id = input
        .strip_prefix('=')
        .unwrap_or(input)
        .parse::<u32>()
        .map_err(|_| format!("Invalid block state ID: {input}"))?;
    let block = Block::from_id(id);
    if block.get_id() != id {
        return Err(format!("Unsupported block state ID: {id}"));
    }
    Ok(block)
}

fn parse_block_states(list: &str) -> Result<FxHashMap<String, String>, String> {
    let properties = parse_properties(list).map_err(|error| error.to_string())?;
    Ok(owned_states(properties))
}

fn parse_block(input: &str) -> Result<(Block, FxHashMap<String, String>), String> {
    let spec = BlockStateSpec::parse(input).map_err(|error| error.to_string())?;
    let name = spec.name();
    let block = Block::from_name(&name).ok_or_else(|| format!("Unknown block: {name}"))?;
    let states = owned_states(spec.properties);
    validate_states(Some(block), &states)?;
    Ok((block, states))
}

fn owned_states(properties: Vec<(&str, &str)>) -> FxHashMap<String, String> {
    properties
        .into_iter()
        .map(|(name, value)| (name.to_owned(), value.to_owned()))
        .collect()
}

fn validate_states(block: Option<Block>, states: &FxHashMap<String, String>) -> Result<(), String> {
    for (name, value) in states {
        let values = match block {
            Some(block) => block.property_values(name).ok_or_else(|| {
                format!("Block {} does not have property {name}", block.get_name())
            })?,
            None => Block::known_properties()
                .iter()
                .find(|property| property.name == name)
                .map(|property| property.values)
                .ok_or_else(|| format!("Unknown block property: {name}"))?,
        };
        if !values.contains(&value.as_str()) {
            return Err(format!("Property {name} does not accept {value}"));
        }
    }
    Ok(())
}
