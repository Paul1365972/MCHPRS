//! This implements Sponge Schematic Specification versions 2 and 3.
//! https://github.com/SpongePowered/Schematic-Specification/blob/master/versions/schematic-2.md

use anyhow::{bail, ensure, Context, Result};
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::BlockPos;
use mchprs_world::storage::PalettedBitBuffer;
use mchprs_world::{World, MC_DATA_VERSION};
use regex::Regex;
use rustc_hash::FxHashMap;
use serde::Serialize;
use std::fs::{self, File};
use std::path::Path;
use std::sync::LazyLock;

#[derive(Clone, Debug)]
pub struct WorldEditClipboard {
    pub offset_x: i32,
    pub offset_y: i32,
    pub offset_z: i32,
    pub size_x: u32,
    pub size_y: u32,
    pub size_z: u32,
    pub data: PalettedBitBuffer,
    pub block_entities: FxHashMap<BlockPos, BlockEntity>,
}

pub fn create_clipboard<W: World>(
    world: &mut W,
    origin: BlockPos,
    first_pos: BlockPos,
    second_pos: BlockPos,
) -> WorldEditClipboard {
    let start_pos = first_pos.min(second_pos);
    let end_pos = first_pos.max(second_pos);
    let size_x = (end_pos.x - start_pos.x) as u32 + 1;
    let size_y = (end_pos.y - start_pos.y) as u32 + 1;
    let size_z = (end_pos.z - start_pos.z) as u32 + 1;
    let offset = origin - start_pos;
    let mut cb = WorldEditClipboard {
        offset_x: offset.x,
        offset_y: offset.y,
        offset_z: offset.z,
        size_x,
        size_y,
        size_z,
        data: PalettedBitBuffer::new((size_x * size_y * size_z) as usize, 9),
        block_entities: FxHashMap::default(),
    };
    let mut i = 0;
    for y in start_pos.y..=end_pos.y {
        for z in start_pos.z..=end_pos.z {
            for x in start_pos.x..=end_pos.x {
                let pos = BlockPos::new(x, y, z);
                let id = world.get_block_raw(pos);
                let block = world.get_block(BlockPos::new(x, y, z));
                if block.has_block_entity()
                    && let Some(block_entity) = world.get_block_entity(pos)
                {
                    cb.block_entities
                        .insert(pos - start_pos, block_entity.clone());
                }
                cb.data.set_entry(i, id);
                i += 1;
            }
        }
    }
    cb
}

pub fn paste_clipboard<W: World>(
    world: &mut W,
    cb: &WorldEditClipboard,
    pos: BlockPos,
    ignore_air: bool,
) {
    let offset_x = pos.x - cb.offset_x;
    let offset_y = pos.y - cb.offset_y;
    let offset_z = pos.z - cb.offset_z;
    let mut i = 0;
    // This can be made better, but right now it's not D:
    let x_range = offset_x..offset_x + cb.size_x as i32;
    let y_range = offset_y..offset_y + cb.size_y as i32;
    let z_range = offset_z..offset_z + cb.size_z as i32;

    let entries = cb.data.entries();
    // I have no clue if these clones are going to cost anything noticeable.
    'top_loop: for y in y_range {
        for z in z_range.clone() {
            for x in x_range.clone() {
                if i >= entries {
                    break 'top_loop;
                }
                let entry = cb.data.get_entry(i);
                i += 1;
                if ignore_air && entry == 0 {
                    continue;
                }
                world.set_block_raw(BlockPos::new(x, y, z), entry);
            }
        }
    }

    // Send block changes before we send block entity data, otherwise it'll be ignored
    world.flush_block_changes();

    for (pos, block_entity) in &cb.block_entities {
        let new_pos = BlockPos {
            x: pos.x + offset_x,
            y: pos.y + offset_y,
            z: pos.z + offset_z,
        };
        world.set_block_entity(new_pos, block_entity.clone());
    }
}

macro_rules! nbt_as {
    // I'm not sure if path is the right type here.
    // It works though!
    ($e:expr, $p:path) => {
        match $e {
            $p(val) => val,
            _ => bail!(concat!("Could not parse nbt value as ", stringify!($p))),
        }
    };
}

macro_rules! nbt_field {
    ($compound:expr, $name:expr, $variant:ident) => {
        match $compound.get($name) {
            Some(nbt::Value::$variant(value)) => value,
            Some(_) => bail!("field {} must be {}", $name, stringify!($variant)),
            None => bail!("missing field {}", $name),
        }
    };
}

fn parse_block(str: &str) -> Option<Block> {
    static RE: LazyLock<Regex> = LazyLock::new(|| {
        Regex::new(r"^([a-z0-9_.-]+:)?([a-z0-9_./-]+)(?:\[([a-z0-9_=,.-]*)\])?$").unwrap()
    });
    let captures = RE.captures(str)?;
    let namespace = captures.get(1).map_or("minecraft:", |value| value.as_str());
    let block_name = format!("{}{}", namespace, captures.get(2)?.as_str());
    let mut properties = std::collections::HashMap::new();
    if let Some(properties_match) = captures.get(3).filter(|value| !value.as_str().is_empty()) {
        for property in properties_match.as_str().split(',') {
            let (name, value) = property.split_once('=')?;
            if name.is_empty()
                || !name
                    .bytes()
                    .all(|byte| byte.is_ascii_lowercase() || byte.is_ascii_digit() || byte == b'_')
                || value.is_empty()
                || value.contains('=')
                || properties.insert(name, value).is_some()
            {
                return None;
            }
        }
    }
    let Some(mut block) = Block::from_name(&block_name) else {
        return Some(Block::Air);
    };
    block.set_properties(properties.clone());
    let decoded_properties = block.properties();
    for (name, value) in properties {
        if let Some(decoded) = decoded_properties.get(name)
            && decoded != value
        {
            return None;
        }
    }
    if matches!(block, Block::Repeater(repeater) if repeater.delay == 0)
        || matches!(
            block,
            Block::SeaPickle { pickles: 0, .. } | Block::WaterCauldron { level: 0 }
        )
        || Block::from_id(block.get_id()) != block
    {
        return None;
    }
    Some(block)
}

pub fn load_schematic(path: &Path) -> Result<WorldEditClipboard> {
    let mut file = File::open(path)?;
    let nbt = nbt::Blob::from_gzip_reader(&mut file)?;

    load_schematic_nbt(&nbt.content)
}

fn load_schematic_nbt(nbt: &nbt::Map<String, nbt::Value>) -> Result<WorldEditClipboard> {
    let root = if nbt.contains_key("Schematic") {
        nbt_field!(nbt, "Schematic", Compound)
    } else {
        nbt
    };

    let version = *nbt_field!(root, "Version", Int);
    match version {
        2 | 3 => load_schematic_sponge(root, version),
        _ => bail!("unknown schematic version: {}", version),
    }
}

fn read_block_container(
    nbt: &nbt::Map<String, nbt::Value>,
    version: i32,
    size_x: u32,
    size_y: u32,
    size_z: u32,
) -> Result<(PalettedBitBuffer, FxHashMap<BlockPos, BlockEntity>)> {
    use nbt::Value;

    let nbt_palette = nbt_field!(nbt, "Palette", Compound);
    let mut palette: FxHashMap<u32, u32> = FxHashMap::default();
    for (k, v) in nbt_palette {
        let id = *nbt_as!(v, Value::Int);
        ensure!(id >= 0, "negative palette index {id}");
        let block = parse_block(k).with_context(|| format!("error parsing block: {}", k))?;
        ensure!(
            palette.insert(id as u32, block.get_id()).is_none(),
            "duplicate palette index {id}"
        );
    }

    let data_name = match version {
        2 => "BlockData",
        3 => "Data",
        _ => unreachable!(),
    };
    let mut blocks = nbt_field!(nbt, data_name, ByteArray).as_slice();
    let volume = (size_x as usize)
        .checked_mul(size_y as usize)
        .and_then(|area| area.checked_mul(size_z as usize))
        .context("schematic dimensions overflow")?;
    ensure!(
        blocks.len() >= volume,
        "{data_name} has fewer bytes than blocks"
    );

    let mut data = PalettedBitBuffer::new(volume, 9);
    for index in 0..volume {
        let palette_index = read_palette_index(&mut blocks)
            .with_context(|| format!("invalid {data_name} entry {index}"))?;
        let entry = palette.get(&palette_index).with_context(|| {
            format!("{data_name} entry {index} references missing palette index {palette_index}")
        })?;
        data.set_entry(index, *entry);
    }
    ensure!(blocks.is_empty(), "trailing bytes in {data_name}");

    let mut parsed_block_entities = FxHashMap::default();
    if nbt.contains_key("BlockEntities") {
        for (index, block_entity) in nbt_field!(nbt, "BlockEntities", List).iter().enumerate() {
            if let Some((pos, entity)) =
                read_block_entity(block_entity, version, [size_x, size_y, size_z])
                    .with_context(|| format!("invalid BlockEntities entry {index}"))?
            {
                ensure!(
                    parsed_block_entities.insert(pos, entity).is_none(),
                    "duplicate block entity at {pos}"
                );
            }
        }
    }

    Ok((data, parsed_block_entities))
}

fn read_palette_index(bytes: &mut &[i8]) -> Result<u32> {
    let mut index = 0;
    for shift in (0..35).step_by(7) {
        let (byte, remaining) = bytes.split_first().context("truncated palette index")?;
        *bytes = remaining;
        let byte = *byte as u8;
        ensure!(
            shift < 28 || byte < 8,
            "palette index exceeds a nonnegative 32-bit integer"
        );
        index |= u32::from(byte & 0x7f) << shift;
        if byte & 0x80 == 0 {
            return Ok(index);
        }
    }
    bail!("palette index exceeds five bytes")
}

fn read_position(values: &[i32], name: &str) -> Result<BlockPos> {
    let [x, y, z] = values else {
        bail!("{name} must contain exactly three integers");
    };
    Ok(BlockPos::new(*x, *y, *z))
}

fn read_block_entity(
    value: &nbt::Value,
    version: i32,
    dimensions: [u32; 3],
) -> Result<Option<(BlockPos, BlockEntity)>> {
    let entity = nbt_as!(value, nbt::Value::Compound);
    let pos = read_position(nbt_field!(entity, "Pos", IntArray), "Pos")?;
    ensure!(
        [pos.x, pos.y, pos.z]
            .into_iter()
            .zip(dimensions)
            .all(|(coordinate, size)| coordinate >= 0 && (coordinate as u32) < size),
        "block entity position {pos} is outside the schematic"
    );
    let id = if entity.contains_key("Id") {
        nbt_field!(entity, "Id", String)
    } else {
        nbt_field!(entity, "id", String)
    };
    let data = match version {
        2 => entity,
        3 => nbt_field!(entity, "Data", Compound),
        _ => unreachable!(),
    };
    let parsed = BlockEntity::from_nbt(id, data);
    if matches!(
        id.trim_start_matches("minecraft:"),
        "comparator" | "furnace" | "barrel" | "hopper" | "sign"
    ) {
        ensure!(parsed.is_some(), "invalid {id} block entity data");
    }
    Ok(parsed.map(|entity| (pos, entity)))
}

fn load_schematic_sponge(
    nbt: &nbt::Map<String, nbt::Value>,
    version: i32,
) -> Result<WorldEditClipboard> {
    let size_x = u32::from(*nbt_field!(nbt, "Width", Short) as u16);
    let size_z = u32::from(*nbt_field!(nbt, "Length", Short) as u16);
    let size_y = u32::from(*nbt_field!(nbt, "Height", Short) as u16);
    ensure!(
        size_x > 0 && size_y > 0 && size_z > 0,
        "schematic dimensions must be nonzero"
    );

    let mut offset = BlockPos::zero();
    let mut legacy_offset = false;
    if version == 2 {
        // Older versions of WorldEdit put the offset in Metadata
        // These offsets are optional but if present all must be present
        // Its important to check the WEOffset first as both can be present but only the WEOffset is correct
        if nbt.contains_key("Metadata") {
            let metadata = nbt_field!(nbt, "Metadata", Compound);
            if ["WEOffsetX", "WEOffsetY", "WEOffsetZ"]
                .iter()
                .any(|name| metadata.contains_key(*name))
            {
                offset = BlockPos::new(
                    *nbt_field!(metadata, "WEOffsetX", Int),
                    *nbt_field!(metadata, "WEOffsetY", Int),
                    *nbt_field!(metadata, "WEOffsetZ", Int),
                );
                legacy_offset = true;
            }
        }
    }
    if !legacy_offset && nbt.contains_key("Offset") {
        offset = read_position(nbt_field!(nbt, "Offset", IntArray), "Offset")?;
    }
    let offset_x = offset
        .x
        .checked_neg()
        .context("Offset X cannot be represented")?;
    let offset_y = offset
        .y
        .checked_neg()
        .context("Offset Y cannot be represented")?;
    let offset_z = offset
        .z
        .checked_neg()
        .context("Offset Z cannot be represented")?;

    let (data, block_entities) = read_block_container(
        match version {
            2 => nbt,
            3 => nbt_field!(nbt, "Blocks", Compound),
            _ => unreachable!(),
        },
        version,
        size_x,
        size_y,
        size_z,
    )?;
    Ok(WorldEditClipboard {
        size_x,
        size_y,
        size_z,
        offset_x,
        offset_y,
        offset_z,
        data,
        block_entities,
    })
}

#[derive(Serialize)]
struct Metadata {
    #[serde(rename = "WEOffsetX")]
    offset_x: i32,
    #[serde(rename = "WEOffsetY")]
    offset_y: i32,
    #[serde(rename = "WEOffsetZ")]
    offset_z: i32,
}

/// Used to serialize schematics in NBT. This cannot be used for deserialization because of
/// [a bug](https://github.com/PistonDevelopers/hematite_nbt/issues/45) in `hematite-nbt`.
#[derive(Serialize)]
#[serde(rename_all = "PascalCase")]
struct Schematic {
    width: i16,
    length: i16,
    height: i16,
    palette: nbt::Blob,
    metadata: Metadata,
    #[serde(serialize_with = "nbt::i8_array")]
    block_data: Vec<i8>,
    block_entities: Vec<nbt::Blob>,
    version: i32,
    data_version: i32,
}

pub fn save_schematic(path: &Path, clipboard: &WorldEditClipboard) -> Result<()> {
    fs::create_dir_all(path.parent().unwrap())?;

    let mut file = File::create(path)?;
    let size_x = clipboard.size_x;
    let size_y = clipboard.size_y;
    let size_z = clipboard.size_z;
    let offset_x = -clipboard.offset_x;
    let offset_y = -clipboard.offset_y;
    let offset_z = -clipboard.offset_z;
    let blocks = &clipboard.data;

    let mut data = Vec::new();
    let mut pallette = Vec::new();
    for y_offset in (0..size_y).map(|y| y * size_z * size_x) {
        for z_offset in (0..size_z).map(|z| z * size_x) {
            for x in 0..size_x {
                let entry = blocks.get_entry((y_offset + z_offset + x) as usize);
                let block = Block::from_id(entry);

                let name = block.get_name();
                let props = block.properties();
                let full_name = if !props.is_empty() {
                    let props_strs: Vec<String> = props
                        .iter()
                        .map(|(name, val)| format!("{}={}", name, val))
                        .collect();
                    format!("{}[{}]", name, props_strs.join(","))
                } else {
                    name.to_owned()
                };
                let mut idx = if let Some(idx) = pallette.iter().position(|s| *s == full_name) {
                    idx
                } else {
                    let idx = pallette.len();
                    pallette.push(full_name);
                    idx
                };

                loop {
                    let mut temp = (idx & 0b1111_1111) as u8;
                    idx >>= 7;
                    if idx != 0 {
                        temp |= 0b1000_0000;
                    }
                    data.push(temp as i8);
                    if idx == 0 {
                        break;
                    }
                }
            }
        }
    }

    let mut encoded_pallete = nbt::Blob::new();
    for (i, entry) in pallette.iter().enumerate() {
        encoded_pallete.insert(entry, i as i32)?;
    }

    let mut block_entities = Vec::new();
    for (pos, block_entity) in &clipboard.block_entities {
        if let Some(mut blob) = block_entity.to_nbt(false) {
            blob.insert("Pos", nbt::Value::IntArray(vec![pos.x, pos.y, pos.z]))?;
            block_entities.push(blob);
        }
    }

    let metadata = Metadata {
        offset_x,
        offset_y,
        offset_z,
    };
    let schematic = Schematic {
        width: size_x as i16,
        length: size_z as i16,
        height: size_y as i16,
        block_data: data,
        block_entities,
        palette: encoded_pallete,
        metadata,
        version: 2,
        data_version: MC_DATA_VERSION,
    };
    nbt::to_gzip_writer(&mut file, &schematic, Some("Schematic"))?;

    Ok(())
}
