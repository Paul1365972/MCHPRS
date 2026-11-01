use crate::{Schematic, SchematicError};
use fastnbt::{DeOpts, Value};
use flate2::read::GzDecoder;
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_state_spec::BlockStateSpec;
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use std::collections::{BTreeMap, HashMap};
use std::io::Read;
use tracing::warn;

type Compound = HashMap<String, Value>;

macro_rules! nbt_field {
    ($compound:expr, $name:expr, $variant:ident) => {
        match $compound.get($name) {
            Some(Value::$variant(value)) => value,
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

pub fn read(bytes: &[u8]) -> Result<Schematic, SchematicError> {
    let mut decompressed = Vec::new();
    GzDecoder::new(bytes).read_to_end(&mut decompressed)?;
    let nbt: Compound =
        fastnbt::from_bytes_with_opts(&decompressed, DeOpts::new().max_seq_len(usize::MAX))?;
    let root = if nbt.contains_key("Schematic") {
        nbt_field!(nbt, "Schematic", Compound)
    } else {
        &nbt
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
    // Older versions of WorldEdit put the offset in Metadata
    // These offsets are optional but if present all must be present
    // Its important to check the WEOffset first as both can be present but only the WEOffset is correct
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
        let Value::Int(index) = value else {
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
    let mut data: &[i8] = nbt_field!(nbt, data_name, ByteArray);
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
                let pos = BlockPos::new(x, y, z);
                let palette_index = read_palette_index(&mut data)
                    .map_err(within(|| format!("invalid {data_name} entry at {pos}")))?;
                let (name, block) = palette.get(&palette_index).ok_or_else(|| {
                    malformed(format!(
                        "{data_name} entry at {pos} references missing palette index \
                         {palette_index}"
                    ))
                })?;
                match block {
                    Some(block) => {
                        volume.set_block(pos, *block);
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
            let Some((pos, block_entity)) = read_block_entity(block_entity, version, region)
                .map_err(within(|| format!("invalid BlockEntities entry {index}")))?
            else {
                continue;
            };
            let pos = region.min() + pos;
            if volume.block_entity(pos).is_some() {
                return Err(malformed(format!("duplicate block entity at {pos}")));
            }
            volume.set_block_entity(pos, Some(block_entity));
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
    let invalid = || malformed(format!("invalid block state {state}"));
    let spec = BlockStateSpec::parse(state).map_err(|_| invalid())?;
    let made_of = |text: &str, extra: &[u8]| {
        text.bytes().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || b"_.-".contains(&byte)
                || extra.contains(&byte)
        })
    };
    let property_name = |name: &str| {
        name.bytes()
            .all(|byte| byte.is_ascii_lowercase() || byte.is_ascii_digit() || byte == b'_')
    };
    if spec.namespace.is_empty()
        || spec.path.is_empty()
        || !made_of(spec.namespace, b"")
        || !made_of(spec.path, b"/")
        || !spec
            .properties
            .iter()
            .all(|&(name, value)| property_name(name) && made_of(value, b""))
    {
        return Err(invalid());
    }
    let Some(mut block) = Block::from_name(&spec.name()) else {
        return Ok(None);
    };
    if spec.properties.iter().any(|&(name, value)| {
        block
            .property_values(name)
            .is_some_and(|values| !values.contains(&value))
    }) {
        return Ok(None);
    }
    block.set_properties(spec.properties.into_iter().collect());
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

fn read_block_entity(
    value: &Value,
    version: i32,
    region: Region,
) -> Result<Option<(BlockPos, BlockEntity)>, SchematicError> {
    let Value::Compound(entity) = value else {
        return Err(malformed("block entity must be Compound"));
    };
    let pos = read_position(nbt_field!(entity, "Pos", IntArray), "Pos")?;
    let local_region = Region::new(BlockPos::zero(), region.max() - region.min());
    if !local_region.contains(pos) {
        return Err(malformed(format!(
            "block entity position {pos} is outside the schematic"
        )));
    }
    let id = if entity.contains_key("Id") {
        nbt_field!(entity, "Id", String)
    } else {
        nbt_field!(entity, "id", String)
    };
    let empty = Compound::new();
    let data = match version {
        2 => entity,
        _ if entity.contains_key("Data") => nbt_field!(entity, "Data", Compound),
        _ => &empty,
    };
    let parsed = BlockEntity::from_nbt(id, data);
    if parsed.is_none() && BlockEntity::is_supported(id) {
        warn!("Skipping invalid {id} block entity data at {pos}");
    }
    Ok(parsed.map(|block_entity| (pos, block_entity)))
}

fn read_position(values: &[i32], name: &str) -> Result<BlockPos, SchematicError> {
    let [x, y, z] = values else {
        return Err(malformed(format!(
            "{name} must contain exactly three integers"
        )));
    };
    Ok(BlockPos::new(*x, *y, *z))
}

fn within(context: impl FnOnce() -> String) -> impl FnOnce(SchematicError) -> SchematicError {
    move |error| match error {
        SchematicError::Malformed(message) => malformed(format!("{}: {message}", context())),
        other => other,
    }
}

fn malformed(message: impl Into<String>) -> SchematicError {
    SchematicError::Malformed(message.into())
}
