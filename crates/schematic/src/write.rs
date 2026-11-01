use crate::SchematicError;
use fastnbt::{ByteArray, IntArray, SerOpts, Value};
use flate2::write::GzEncoder;
use flate2::Compression;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::{BlockPos, MC_DATA_VERSION};
use serde::Serialize;
use std::collections::HashMap;

#[derive(Serialize)]
struct Metadata {
    #[serde(rename = "WEOffsetX")]
    offset_x: i32,
    #[serde(rename = "WEOffsetY")]
    offset_y: i32,
    #[serde(rename = "WEOffsetZ")]
    offset_z: i32,
}

#[derive(Serialize)]
#[serde(rename_all = "PascalCase")]
struct Schematic {
    width: i16,
    length: i16,
    height: i16,
    palette: HashMap<String, i32>,
    metadata: Metadata,
    block_data: ByteArray,
    block_entities: Vec<HashMap<String, Value>>,
    version: i32,
    data_version: i32,
}

pub fn write(volume: &BlockVolume) -> Result<Vec<u8>, SchematicError> {
    let region = volume.region();
    let size = |min: i32, max: i32| {
        u16::try_from(i64::from(max) - i64::from(min) + 1)
            .ok()
            .filter(|size| *size > 0)
            .map(|size| size as i16)
            .ok_or(SchematicError::UnsupportedSize)
    };
    let width = size(region.min().x, region.max().x)?;
    let height = size(region.min().y, region.max().y)?;
    let length = size(region.min().z, region.max().z)?;

    let mut block_data = Vec::new();
    let mut palette = HashMap::new();
    for y in region.min().y..=region.max().y {
        for z in region.min().z..=region.max().z {
            for x in region.min().x..=region.max().x {
                let block = volume.block(BlockPos::new(x, y, z));
                let next_index = palette.len();
                let mut index = *palette.entry(block.get_id()).or_insert(next_index);
                loop {
                    let mut byte = (index & 0b0111_1111) as u8;
                    index >>= 7;
                    if index != 0 {
                        byte |= 0b1000_0000;
                    }
                    block_data.push(byte as i8);
                    if index == 0 {
                        break;
                    }
                }
            }
        }
    }

    let encoded_palette = palette
        .into_iter()
        .map(|(id, index)| (block_state(Block::from_id(id)), index as i32))
        .collect();

    let mut block_entities = Vec::new();
    for (position, block_entity) in volume.block_entities() {
        if let Some(mut nbt) = block_entity.to_nbt(false) {
            let id = nbt.remove("id").expect("block entity NBT has an id");
            nbt.insert("Id".to_owned(), id);
            let position = position - region.min();
            nbt.insert(
                "Pos".to_owned(),
                Value::IntArray(IntArray::new(vec![position.x, position.y, position.z])),
            );
            block_entities.push(nbt);
        }
    }

    let schematic = Schematic {
        width,
        length,
        height,
        palette: encoded_palette,
        metadata: Metadata {
            offset_x: region.min().x,
            offset_y: region.min().y,
            offset_z: region.min().z,
        },
        block_data: ByteArray::new(block_data),
        block_entities,
        version: 2,
        data_version: MC_DATA_VERSION,
    };
    let mut encoder = GzEncoder::new(Vec::new(), Compression::default());
    fastnbt::to_writer_with_opts(
        &mut encoder,
        &schematic,
        SerOpts::new().root_name("Schematic"),
    )?;
    Ok(encoder.finish()?)
}

fn block_state(block: Block) -> String {
    let properties = block.properties();
    if properties.is_empty() {
        return block.get_name().to_owned();
    }
    let properties: Vec<String> = properties
        .iter()
        .map(|(name, value)| format!("{name}={value}"))
        .collect();
    format!("{}[{}]", block.get_name(), properties.join(","))
}
