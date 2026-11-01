use anyhow::Result;
use fastnbt::{LongArray, Value};
use flate2::write::ZlibEncoder;
use flate2::Compression;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::BlockPos;
use mchprs_blocks::MC_DATA_VERSION;
use mchprs_save_data::plot_data::{ChunkData, PlotData};
use mchprs_world::storage::{ChunkSection, PalettedBitBuffer};
use rayon::iter::{IndexedParallelIterator, IntoParallelIterator, ParallelIterator};
use std::collections::HashMap;
use std::fs;
use std::path::Path;

fn encode_section(section: &ChunkSection) -> HashMap<String, Value> {
    let num_entries = 16 * 16 * 16;
    // Copy the chunk section's internal buffer
    let orig_buffer = PalettedBitBuffer::load(
        num_entries,
        section.bits_per_block(),
        section.data().to_vec(),
        section.palette().to_vec(),
        9,
    );

    // Copy into a new buffer that can never use direct palette
    let mut buffer = PalettedBitBuffer::new(num_entries, u64::MAX);
    for idx in 0..num_entries {
        buffer.set_entry(idx, orig_buffer.get_entry(idx));
    }

    let mut palette = Vec::new();
    for block in buffer.palette() {
        let block = Block::from_id(*block);
        let name = block.get_name();
        let props = block.properties();
        let props = props
            .iter()
            .map(|(name, val)| (name.to_string(), Value::String(val.to_string())))
            .collect();
        let entry = Value::Compound(HashMap::from([
            ("Name".to_owned(), Value::String(name.to_string())),
            ("Properties".to_owned(), Value::Compound(props)),
        ]));
        palette.push(entry);
    }

    HashMap::from([
        ("palette".to_owned(), Value::List(palette)),
        (
            "data".to_owned(),
            Value::LongArray(LongArray::new(
                buffer.data().iter().map(|val| *val as i64).collect(),
            )),
        ),
    ])
}

fn serialize_chunk(chunk_x: i32, chunk_z: i32, chunk: ChunkData) -> Result<Vec<u8>> {
    let chunk = chunk.load(chunk_x, chunk_z);
    let mut nbt = HashMap::from([
        ("DataVersion".to_owned(), Value::Int(MC_DATA_VERSION)),
        ("xPos".to_owned(), Value::Int(chunk_x)),
        ("zPos".to_owned(), Value::Int(chunk_z)),
        ("yPos".to_owned(), Value::Int(0)),
        (
            "Status".to_owned(),
            Value::String("minecraft:full".to_owned()),
        ),
        ("LastUpdate".to_owned(), Value::Long(0)),
    ]);

    let mut block_entities = Vec::new();
    for (pos, block_entity) in &chunk.block_entities {
        let pos = BlockPos {
            x: pos.x + chunk_x * 16,
            z: pos.z + chunk_z * 16,
            ..*pos
        };
        let mut block_entity_nbt = block_entity.to_nbt(false).unwrap();
        block_entity_nbt.insert("x".to_owned(), Value::Int(pos.x));
        block_entity_nbt.insert("y".to_owned(), Value::Int(pos.y));
        block_entity_nbt.insert("z".to_owned(), Value::Int(pos.z));
        block_entities.push(Value::Compound(block_entity_nbt));
    }
    nbt.insert("block_entities".to_owned(), Value::List(block_entities));

    let mut sections = Vec::new();
    for (section_y, section) in chunk.sections.iter().enumerate() {
        let mut section_nbt = HashMap::new();
        section_nbt.insert("Y".to_string(), Value::Byte(section_y as i8));
        let block_states = if section.block_count() != 0 {
            encode_section(section)
        } else {
            let air = Value::Compound(HashMap::from([(
                "Name".to_owned(),
                Value::String("minecraft:air".to_string()),
            )]));
            HashMap::from([("palette".to_owned(), Value::List(vec![air]))])
        };
        section_nbt.insert("block_states".to_string(), Value::Compound(block_states));
        sections.push(Value::Compound(section_nbt));
    }
    nbt.insert("sections".to_owned(), Value::List(sections));

    let mut encoder = ZlibEncoder::new(vec![0, 0, 0, 0, 2], Compression::default());
    fastnbt::to_writer(&mut encoder, &nbt)?;
    let mut data = encoder.finish()?;
    let len = data.len() - 5;
    data[0] = ((len >> 24) & 0xFF) as u8;
    data[1] = ((len >> 16) & 0xFF) as u8;
    data[2] = ((len >> 8) & 0xFF) as u8;
    data[3] = (len & 0xFF) as u8;
    Ok(data)
}

#[derive(Default)]
struct Region {
    chunks: HashMap<(u8, u8), Vec<u8>>,
}

pub fn generate_regions(world_path: &Path, output_path: &Path) -> Result<()> {
    let plots_path = world_path.join("plots");
    let plots_dir = fs::read_dir(plots_path)?;

    let mut regions: HashMap<(i32, i32), Region> = HashMap::new();

    let mut last_plot_chunk_width = None;
    for dir_entry in plots_dir {
        let dir_entry = dir_entry?;
        if !dir_entry.file_type()?.is_file() {
            continue;
        }
        let Ok(file_name) = dir_entry.file_name().into_string() else {
            continue;
        };
        let Some((plot_x, plot_z)) = file_name
            .strip_prefix('p')
            .and_then(|name| name.split_once(','))
            .and_then(|(x, y)| Some((x.parse::<i32>().ok()?, y.parse::<i32>().ok()?)))
        else {
            continue;
        };

        let plot_data = PlotData::load_from_file(dir_entry.path())?;
        let plot_chunk_width = (plot_data.chunk_data.len() as f64).sqrt() as i32;
        let last_plot_chunk_width = last_plot_chunk_width.replace(plot_chunk_width);
        if last_plot_chunk_width.is_some() && last_plot_chunk_width != Some(plot_chunk_width) {
            panic!("Found plot scale mismatch");
        }

        println!("processing plot file: {}", file_name);

        let chunks = plot_data
            .chunk_data
            .into_par_iter()
            .enumerate()
            .map(|(chunk_idx, chunk_data)| {
                let chunk_idx = chunk_idx as i32;
                let chunk_x = chunk_idx / plot_chunk_width + plot_x * plot_chunk_width;
                let chunk_z = chunk_idx % plot_chunk_width + plot_z * plot_chunk_width;
                let region_pos = (chunk_x >> 5, chunk_z >> 5);
                let pos_in_region = ((chunk_x & 31) as u8, (chunk_z & 31) as u8);
                let data = serialize_chunk(chunk_x, chunk_z, chunk_data)?;
                Ok((region_pos, pos_in_region, data))
            })
            .collect::<Result<Vec<_>>>()?;
        for (region_pos, pos_in_region, data) in chunks {
            let region = regions.entry(region_pos).or_default();
            region.chunks.insert(pos_in_region, data);
        }
    }

    let region_path = output_path.join("region");
    fs::create_dir(&region_path)?;

    for ((region_x, region_z), region) in regions {
        let mut region_data = vec![0u8; 0x2000];
        let mut next_sector = 2;
        for ((chunk_x, chunk_z), mut chunk_data) in region.chunks {
            let sector_count = chunk_data.len().div_ceil(0x1000) as u8;
            let sector_offset = next_sector;
            let header_offset = 4 * (chunk_x as usize + chunk_z as usize * 32);
            region_data[header_offset] = ((sector_offset >> 16) & 0xFF) as u8;
            region_data[header_offset + 1] = ((sector_offset >> 8) & 0xFF) as u8;
            region_data[header_offset + 2] = (sector_offset & 0xFF) as u8;
            region_data[header_offset + 3] = sector_count;

            next_sector += sector_count as u32;
            let padding = sector_count as usize * 0x1000 - chunk_data.len();
            region_data.append(&mut chunk_data);
            region_data.extend(std::iter::repeat_n(0, padding));
        }
        let file_name = format!("r.{}.{}.mca", region_x, region_z);
        fs::write(region_path.join(&file_name), region_data)?;
        println!("wrote region file: {}", file_name);
    }

    Ok(())
}
