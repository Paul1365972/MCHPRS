use crate::{Schematic, SchematicError};
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use regex::Regex;
use std::collections::{BTreeMap, HashMap};
use std::sync::LazyLock;
use tracing::warn;

type Compound = nbt::Map<String, nbt::Value>;

macro_rules! nbt_field {
    ($compound:expr, $name:expr, $variant:ident) => {
        match $compound.get($name) {
            Some(nbt::Value::$variant(value)) => value,
            Some(_) => {
                return Err(malformed(format!(
                    "field {} must be {}",
                    $name,
                    stringify!($variant)
                )))
            }
            None => return Err(malformed(format!("missing field {}", $name))),
        }
    };
}

fn malformed(message: impl Into<String>) -> SchematicError {
    SchematicError::Malformed(message.into())
}

fn within(context: impl FnOnce() -> String) -> impl FnOnce(SchematicError) -> SchematicError {
    move |error| match error {
        SchematicError::Malformed(message) => malformed(format!("{}: {message}", context())),
        other => other,
    }
}

pub fn read(mut bytes: &[u8]) -> Result<Schematic, SchematicError> {
    let nbt = nbt::Blob::from_gzip_reader(&mut bytes)?;
    let root = if nbt.content.contains_key("Schematic") {
        nbt_field!(nbt.content, "Schematic", Compound)
    } else {
        &nbt.content
    };
    let version = *nbt_field!(root, "Version", Int);
    match version {
        2 | 3 => read_sponge(root, version),
        _ => Err(SchematicError::UnsupportedVersion(version)),
    }
}

fn read_sponge(nbt: &Compound, version: i32) -> Result<Schematic, SchematicError> {
    let size_x = u32::from(*nbt_field!(nbt, "Width", Short) as u16);
    let size_y = u32::from(*nbt_field!(nbt, "Height", Short) as u16);
    let size_z = u32::from(*nbt_field!(nbt, "Length", Short) as u16);
    if size_x == 0 || size_y == 0 || size_z == 0 {
        return Err(malformed("schematic dimensions must be nonzero"));
    }

    let min = read_offset(nbt, version)?;
    let corner = |start: i32, size: u32| {
        start
            .checked_add(size as i32 - 1)
            .ok_or_else(|| malformed("Offset places the schematic beyond the coordinate range"))
    };
    let region = Region::new(
        min,
        BlockPos::new(
            corner(min.x, size_x)?,
            corner(min.y, size_y)?,
            corner(min.z, size_z)?,
        ),
    );

    let blocks = match version {
        2 => nbt,
        _ => nbt_field!(nbt, "Blocks", Compound),
    };
    read_block_container(blocks, version, region)
}

fn read_offset(nbt: &Compound, version: i32) -> Result<BlockPos, SchematicError> {
    // Older WorldEdit versions write all three WEOffset fields to Metadata, which win over Offset
    if version == 2 && nbt.contains_key("Metadata") {
        let metadata = nbt_field!(nbt, "Metadata", Compound);
        if ["WEOffsetX", "WEOffsetY", "WEOffsetZ"]
            .iter()
            .any(|name| metadata.contains_key(*name))
        {
            return Ok(BlockPos::new(
                *nbt_field!(metadata, "WEOffsetX", Int),
                *nbt_field!(metadata, "WEOffsetY", Int),
                *nbt_field!(metadata, "WEOffsetZ", Int),
            ));
        }
    }
    if nbt.contains_key("Offset") {
        return read_position(nbt_field!(nbt, "Offset", IntArray), "Offset");
    }
    Ok(BlockPos::zero())
}

fn read_block_container(
    nbt: &Compound,
    version: i32,
    region: Region,
) -> Result<Schematic, SchematicError> {
    let mut palette = HashMap::new();
    for (name, value) in nbt_field!(nbt, "Palette", Compound) {
        let nbt::Value::Int(index) = value else {
            return Err(malformed(format!("palette index of {name} must be Int")));
        };
        if *index < 0 {
            return Err(malformed(format!("negative palette index {index}")));
        }
        if palette
            .insert(*index as u32, (name.as_str(), parse_block(name)?))
            .is_some()
        {
            return Err(malformed(format!("duplicate palette index {index}")));
        }
    }

    let data_name = match version {
        2 => "BlockData",
        _ => "Data",
    };
    let mut data = nbt_field!(nbt, data_name, ByteArray).as_slice();
    let size = region.max() - region.min();
    let [size_x, size_y, size_z] = [size.x, size.y, size.z].map(|size| size as usize + 1);
    let block_count = size_x
        .checked_mul(size_y)
        .and_then(|area| area.checked_mul(size_z))
        .ok_or_else(|| malformed("schematic dimensions overflow"))?;
    if data.len() < block_count {
        return Err(malformed(format!(
            "{data_name} has fewer bytes than blocks"
        )));
    }

    let mut volume = BlockVolume::empty(region);
    let mut unknown_blocks = BTreeMap::new();
    let (min, max) = (region.min(), region.max());
    for y in min.y..=max.y {
        for z in min.z..=max.z {
            for x in min.x..=max.x {
                let position = BlockPos::new(x, y, z);
                let palette_index = read_palette_index(&mut data).map_err(within(|| {
                    format!("invalid {data_name} entry at {position}")
                }))?;
                let (name, block) = palette.get(&palette_index).ok_or_else(|| {
                    malformed(format!(
                        "{data_name} entry at {position} references missing palette index {palette_index}"
                    ))
                })?;
                match block {
                    Some(block) => {
                        volume.set_block(position, *block);
                    }
                    None => *unknown_blocks.entry(*name).or_insert(0) += 1,
                }
            }
        }
    }
    if !data.is_empty() {
        return Err(malformed(format!("trailing bytes in {data_name}")));
    }

    if nbt.contains_key("BlockEntities") {
        for (index, block_entity) in nbt_field!(nbt, "BlockEntities", List).iter().enumerate() {
            let Some((position, block_entity)) =
                read_block_entity(block_entity, version, region)
                    .map_err(within(|| format!("invalid BlockEntities entry {index}")))?
            else {
                continue;
            };
            let position = region.min() + position;
            if volume.block_entity(position).is_some() {
                return Err(malformed(format!("duplicate block entity at {position}")));
            }
            volume.set_block_entity(position, Some(block_entity));
        }
    }

    Ok(Schematic {
        volume,
        unknown_blocks: unknown_blocks
            .into_iter()
            .map(|(name, count)| (name.to_owned(), count))
            .collect(),
    })
}

fn parse_block(state: &str) -> Result<Option<Block>, SchematicError> {
    static BLOCK_STATE_PATTERN: LazyLock<Regex> = LazyLock::new(|| {
        Regex::new(r"^([a-z0-9_.-]+:)?([a-z0-9_./-]+)(?:\[([a-z0-9_=,.-]*)\])?$").unwrap()
    });
    let invalid = || malformed(format!("invalid block state {state}"));
    let captures = BLOCK_STATE_PATTERN.captures(state).ok_or_else(invalid)?;
    let namespace = captures.get(1).map_or("minecraft:", |value| value.as_str());
    let path = captures.get(2).ok_or_else(invalid)?.as_str();
    let block_name = format!("{namespace}{path}");
    let mut properties = HashMap::new();
    if let Some(properties_match) = captures.get(3).filter(|value| !value.as_str().is_empty()) {
        for property in properties_match.as_str().split(',') {
            let (name, value) = property.split_once('=').ok_or_else(invalid)?;
            if name.is_empty()
                || !name
                    .bytes()
                    .all(|byte| byte.is_ascii_lowercase() || byte.is_ascii_digit() || byte == b'_')
                || value.is_empty()
                || value.contains('=')
                || properties.insert(name, value).is_some()
            {
                return Err(invalid());
            }
        }
    }
    let Some(mut block) = Block::from_name(&block_name) else {
        return Ok(None);
    };
    if properties.iter().any(|(name, value)| {
        block
            .property_values(name)
            .is_some_and(|values| !values.contains(value))
    }) {
        return Ok(None);
    }
    block.set_properties(properties);
    Ok(Some(block))
}

fn read_palette_index(bytes: &mut &[i8]) -> Result<u32, SchematicError> {
    let mut index = 0;
    for shift in (0..35).step_by(7) {
        let (byte, remaining) = bytes
            .split_first()
            .ok_or_else(|| malformed("truncated palette index"))?;
        *bytes = remaining;
        let byte = *byte as u8;
        if shift >= 28 && byte >= 8 {
            return Err(malformed(
                "palette index exceeds a nonnegative 32-bit integer",
            ));
        }
        index |= u32::from(byte & 0x7f) << shift;
        if byte & 0x80 == 0 {
            return Ok(index);
        }
    }
    Err(malformed("palette index exceeds five bytes"))
}

fn read_position(values: &[i32], name: &str) -> Result<BlockPos, SchematicError> {
    let [x, y, z] = values else {
        return Err(malformed(format!(
            "{name} must contain exactly three integers"
        )));
    };
    Ok(BlockPos::new(*x, *y, *z))
}

fn read_block_entity(
    value: &nbt::Value,
    version: i32,
    region: Region,
) -> Result<Option<(BlockPos, BlockEntity)>, SchematicError> {
    let nbt::Value::Compound(entity) = value else {
        return Err(malformed("block entity must be Compound"));
    };
    let position = read_position(nbt_field!(entity, "Pos", IntArray), "Pos")?;
    let local_region = Region::new(BlockPos::zero(), region.max() - region.min());
    if !local_region.contains(position) {
        return Err(malformed(format!(
            "block entity position {position} is outside the schematic"
        )));
    }
    let id = if entity.contains_key("Id") {
        nbt_field!(entity, "Id", String)
    } else {
        nbt_field!(entity, "id", String)
    };
    let empty = nbt::Map::new();
    let data = match version {
        2 => entity,
        _ if entity.contains_key("Data") => nbt_field!(entity, "Data", Compound),
        _ => &empty,
    };
    let parsed = BlockEntity::from_nbt(id, data);
    if parsed.is_none() && BlockEntity::is_supported(id) {
        warn!("Skipping invalid {id} block entity data at {position}");
    }
    Ok(parsed.map(|block_entity| (position, block_entity)))
}
