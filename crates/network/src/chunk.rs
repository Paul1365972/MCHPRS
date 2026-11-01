use crate::nbt_util::NBTCompound;
use crate::packets::clientbound::{
    CChunkData, CChunkDataBlockEntity, CChunkDataSection, CUpdateSectionBlocks,
    CUpdateSectionBlocksRecord, ClientBoundPacket,
};
use crate::packets::{PacketEncoder, PalettedContainer};
use fastnbt::{LongArray, Value};
use mchprs_world::storage::{BitBuffer, Chunk, ChunkSection};

pub fn encode_chunk(chunk: &Chunk) -> PacketEncoder {
    let block_height = chunk.sections.len() * 16;
    // Integer arithmetic trick: ceil(log2(x)) can be calculated with 32 - (x -
    // 1).leading_zeros(). See also: https://wiki.vg/Protocol#Chunk_Data_and_Update_Light
    let heightmap_bits = (32 - ((block_height as u32 + 1) - 1).leading_zeros()) as u8;
    let mut heightmap_buffer = BitBuffer::create(heightmap_bits, 16 * 16);
    for z in 0..16 {
        for x in 0..16 {
            heightmap_buffer.set_entry(z * 16 + x, column_height(chunk, x as u32, z as u32));
        }
    }

    let chunk_sections = chunk.sections.iter().map(encode_section).collect();
    let heightmap_longs: Vec<i64> = heightmap_buffer.longs().iter().map(|x| *x as i64).collect();
    let heightmaps = NBTCompound::from([(
        "MOTION_BLOCKING".to_owned(),
        Value::LongArray(LongArray::new(heightmap_longs)),
    )]);
    let mut block_entities = Vec::new();
    for (pos, block_entity) in &chunk.block_entities {
        if let Some(nbt) = block_entity.to_nbt(true) {
            block_entities.push(CChunkDataBlockEntity {
                x: pos.x as i8,
                z: pos.z as i8,
                y: pos.y as i16,
                data: nbt,
                ty: block_entity.ty(),
            })
        }
    }
    CChunkData {
        chunk_sections,
        chunk_x: chunk.x,
        chunk_z: chunk.z,
        heightmaps,
        block_entities,
    }
    .encode()
}

pub fn encode_empty_chunk(x: i32, z: i32, section_count: usize) -> PacketEncoder {
    CChunkData {
        chunk_sections: (0..section_count)
            .map(|_| CChunkDataSection {
                block_count: 0,
                block_states: PalettedContainer {
                    bits_per_entry: 0,
                    data_array: vec![0],
                    palette: Some(vec![0]),
                },
                biomes: PalettedContainer {
                    bits_per_entry: 0,
                    data_array: vec![0],
                    palette: Some(vec![0]),
                },
            })
            .collect(),
        chunk_x: x,
        chunk_z: z,
        heightmaps: NBTCompound::new(),
        block_entities: vec![],
    }
    .encode()
}

pub fn encode_block_changes(chunk: &mut Chunk) -> Vec<PacketEncoder> {
    let (chunk_x, chunk_z) = (chunk.x, chunk.z);
    chunk
        .sections
        .iter_mut()
        .enumerate()
        .filter_map(|(section_y, section)| {
            let records: Vec<_> = section
                .drain_changes()
                .into_iter()
                .map(|change| CUpdateSectionBlocksRecord {
                    block_id: change.block_state,
                    x: change.x,
                    y: change.y,
                    z: change.z,
                })
                .collect();
            (!records.is_empty()).then(|| {
                CUpdateSectionBlocks {
                    chunk_x,
                    chunk_y: section_y as u32,
                    chunk_z,
                    records,
                }
                .encode()
            })
        })
        .collect()
}

fn encode_section(section: &ChunkSection) -> CChunkDataSection {
    let block_states = if section.uses_palette() && section.palette().len() == 1 {
        PalettedContainer {
            bits_per_entry: 0,
            data_array: vec![0],
            palette: Some(vec![section.palette()[0] as i32]),
        }
    } else {
        PalettedContainer {
            bits_per_entry: section.bits_per_block(),
            data_array: section.data().to_vec(),
            palette: section
                .uses_palette()
                .then(|| section.palette().iter().map(|x| *x as i32).collect()),
        }
    };
    CChunkDataSection {
        block_count: section.block_count() as i16,
        block_states,
        biomes: PalettedContainer {
            bits_per_entry: 0,
            data_array: vec![],
            palette: Some(vec![0]),
        },
    }
}

fn column_height(chunk: &Chunk, x: u32, z: u32) -> u32 {
    for (section_y, section) in chunk.sections.iter().enumerate().rev() {
        if section.block_count() == 0 {
            continue;
        }
        if let Some(y) = (0..16).rev().find(|&y| section.get_block(x, y, z) != 0) {
            return section_y as u32 * 16 + y + 1;
        }
    }
    0
}
