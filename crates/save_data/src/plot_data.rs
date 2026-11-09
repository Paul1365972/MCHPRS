mod v2_to_v3;
mod v3_to_v4;

use crate::replace_file;
use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::BlockStore;
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_redstone::tick_queue::{TickEntry, TickPriority};
use mchprs_world::storage::{Chunk, ChunkSection, PalettedBitBuffer, BLOCK_STATE_DIRECT_THRESHOLD};
use rustc_hash::FxHashMap;
use serde::{Deserialize, Serialize};
use std::borrow::Cow;
use std::fs::{self, File};
use std::io::{BufWriter, Read, Write};
use std::path::Path;
use std::{fmt, io};
use thiserror::Error;
use v2_to_v3::PlotDataV2;
use v3_to_v4::PlotDataV3;

/// Version History:
/// 0: Initial plot data file with header (MC 1.18.2)
/// 1: Add world send rate
/// 2: Update to MC 1.20.4
/// 3: Change Tps and WorldSendRate to support real values (f32)
/// 4: Save the block each pending tick is for
pub const VERSION: u32 = 4;

const WRITE_BUFFER_SIZE: usize = 1 << 20;

#[derive(Error, Debug)]
pub enum PlotLoadError {
    #[error("plot data deserialization error")]
    Deserialize(#[from] bincode::Error),

    #[error("invalid plot data header")]
    InvalidHeader,

    #[error("plot data version {0} too new to be loaded")]
    TooNew(u32),

    #[error(transparent)]
    Io(#[from] io::Error),

    #[error("conversion from plot data version {0} is unavailable")]
    ConversionUnavailable(u32),

    #[error("plot data has unknown block state {0}")]
    UnknownBlockState(u32),
}

#[derive(Error, Debug)]
pub enum PlotSaveError {
    #[error("plot data serialization error: {0}")]
    Serialize(#[from] bincode::Error),

    #[error(transparent)]
    Io(#[from] io::Error),
}

static PLOT_MAGIC: &[u8; 8] = b"\x86MCHPRS\x00";

#[derive(Debug, Serialize, Deserialize, PartialEq, Eq, Clone)]
pub struct ChunkSectionData<'a> {
    pub data: Cow<'a, [u64]>,
    pub palette: Cow<'a, [u32]>,
    pub bits_per_block: u8,
    pub block_count: u32,
}

impl<'a> ChunkSectionData<'a> {
    fn new(section: &'a ChunkSection) -> Self {
        Self {
            data: Cow::Borrowed(section.data()),
            palette: Cow::Borrowed(section.palette()),
            bits_per_block: section.bits_per_block(),
            block_count: section.block_count(),
        }
    }

    fn into_owned(self) -> ChunkSectionData<'static> {
        ChunkSectionData {
            data: Cow::Owned(self.data.into_owned()),
            palette: Cow::Owned(self.palette.into_owned()),
            bits_per_block: self.bits_per_block,
            block_count: self.block_count,
        }
    }

    fn unknown_block_state(&self) -> Option<u32> {
        let buffer = PalettedBitBuffer::load(
            16 * 16 * 16,
            self.bits_per_block,
            self.data.to_vec(),
            self.palette.to_vec(),
            BLOCK_STATE_DIRECT_THRESHOLD,
        );
        let is_unknown = |id: &u32| Block::from_id(*id).is_none();
        if buffer.uses_palette() {
            return buffer.palette().iter().copied().find(is_unknown);
        }
        (0..buffer.entries())
            .map(|idx| buffer.get_entry(idx))
            .find(is_unknown)
    }

    fn load(self) -> ChunkSection {
        ChunkSection::from_raw(
            self.data.into_owned(),
            self.bits_per_block,
            self.palette.into_owned(),
            self.block_count,
        )
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ChunkData<'a> {
    pub sections: Vec<Option<ChunkSectionData<'a>>>,
    pub block_entities: Cow<'a, FxHashMap<BlockPos, BlockEntity>>,
}

impl<'a> ChunkData<'a> {
    pub fn new(chunk: &'a mut Chunk) -> Self {
        chunk.flush();
        let chunk: &'a Chunk = chunk;
        Self {
            sections: chunk
                .sections
                .iter()
                .map(|section| {
                    if section.block_count() > 0 {
                        Some(ChunkSectionData::new(section))
                    } else {
                        None
                    }
                })
                .collect(),
            block_entities: Cow::Borrowed(&chunk.block_entities),
        }
    }

    pub fn into_owned(self) -> ChunkData<'static> {
        ChunkData {
            sections: self
                .sections
                .into_iter()
                .map(|section| section.map(ChunkSectionData::into_owned))
                .collect(),
            block_entities: Cow::Owned(self.block_entities.into_owned()),
        }
    }

    pub fn load(self, x: i32, z: i32) -> Chunk {
        Chunk {
            x,
            z,
            sections: self
                .sections
                .into_iter()
                .map(|section| match section {
                    Some(section) => section.load(),
                    None => Default::default(),
                })
                .collect(),
            block_entities: self.block_entities.into_owned(),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub enum Tps {
    Limited(f32),
    Unlimited,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct WorldSendRate(pub f32);

impl Default for WorldSendRate {
    fn default() -> Self {
        Self(60.0)
    }
}

impl fmt::Display for Tps {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Tps::Limited(tps) => write!(f, "{}", tps),
            Tps::Unlimited => write!(f, "unlimited"),
        }
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SavedTick {
    pub ticks_left: u32,
    pub priority: TickPriority,
    pub pos: BlockPos,
    pub block_id: u32,
}

impl From<&TickEntry> for SavedTick {
    fn from(tick: &TickEntry) -> SavedTick {
        SavedTick {
            ticks_left: tick.ticks_left,
            priority: tick.priority,
            pos: tick.pos,
            block_id: tick.block.get_id(),
        }
    }
}

impl From<SavedTick> for TickEntry {
    fn from(tick: SavedTick) -> TickEntry {
        TickEntry::new(
            tick.pos,
            tick.ticks_left,
            tick.priority,
            Block::from_id(tick.block_id).expect("loaded plot data holds known block states"),
        )
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct PlotData<'a> {
    pub tps: Tps,
    pub world_send_rate: WorldSendRate,
    pub chunk_data: Vec<ChunkData<'a>>,
    pub pending_ticks: Vec<SavedTick>,
}

impl PlotData<'_> {
    pub fn chunk_width(&self) -> i32 {
        chunk_width(self.chunk_data.len())
    }

    pub fn chunk_pos(chunk_width: i32, idx: usize) -> (i32, i32) {
        let idx = idx as i32;
        (idx / chunk_width, idx % chunk_width)
    }

    pub fn chunk_idx(chunk_width: i32, x: i32, z: i32) -> usize {
        (x * chunk_width + z) as usize
    }

    pub fn load_from_file(path: impl AsRef<Path>) -> Result<PlotData<'static>, PlotLoadError> {
        let path = path.as_ref();
        let mut file = File::open(path)?;
        let version = read_version(&mut file)?;
        let mut buf = Vec::new();
        file.read_to_end(&mut buf)?;
        let data = match version {
            0..=1 => return Err(PlotLoadError::ConversionUnavailable(version)),
            2 => bincode::deserialize::<PlotDataV2>(&buf)?
                .upgrade()
                .upgrade(),
            3 => bincode::deserialize::<PlotDataV3>(&buf)?.upgrade(),
            VERSION => bincode::deserialize(&buf)?,
            _ => return Err(PlotLoadError::TooNew(version)),
        };
        match data.unknown_block_state() {
            Some(id) => Err(PlotLoadError::UnknownBlockState(id)),
            None => Ok(data),
        }
    }

    fn unknown_block_state(&self) -> Option<u32> {
        let mut sections = self
            .chunk_data
            .iter()
            .flat_map(|chunk| chunk.sections.iter().flatten());
        let mut ticks = self.pending_ticks.iter().map(|tick| tick.block_id);
        sections
            .find_map(ChunkSectionData::unknown_block_state)
            .or_else(|| ticks.find(|&id| Block::from_id(id).is_none()))
    }

    /// Positions are relative to the plot's corner.
    pub fn into_volume(self) -> (BlockVolume, Vec<TickEntry>) {
        let chunk_width = self.chunk_width().max(1);
        let sections = self.chunk_data.iter().map(|chunk| chunk.sections.len());
        let block_height = sections.max().unwrap_or(0).max(1) * 16;
        let block_width = chunk_width * 16;
        let region = Region::new(
            BlockPos::zero(),
            BlockPos::new(block_width - 1, block_height as i32 - 1, block_width - 1),
        );
        let mut volume = BlockVolume::empty(region);
        for (idx, chunk_data) in self.chunk_data.into_iter().enumerate() {
            let (chunk_x, chunk_z) = Self::chunk_pos(chunk_width, idx);
            let chunk = chunk_data.load(chunk_x, chunk_z);
            for (section_y, section) in chunk.sections.iter().enumerate() {
                if section.block_count() == 0 {
                    continue;
                }
                for y in 0..16 {
                    for z in 0..16 {
                        for x in 0..16 {
                            let id = section.get_block(x, y, z);
                            if id == 0 {
                                continue;
                            }
                            let pos = BlockPos::new(
                                chunk_x * 16 + x as i32,
                                section_y as i32 * 16 + y as i32,
                                chunk_z * 16 + z as i32,
                            );
                            let block = Block::from_id(id)
                                .expect("loaded plot data holds known block states");
                            volume.set_block(pos, block);
                        }
                    }
                }
            }
            for (local, block_entity) in chunk.block_entities {
                let pos = BlockPos::new(chunk_x * 16 + local.x, local.y, chunk_z * 16 + local.z);
                volume.set_block_entity(pos, Some(block_entity));
            }
        }
        let ticks = self
            .pending_ticks
            .into_iter()
            .map(|tick| {
                let pos = BlockPos::new(
                    tick.pos.x.rem_euclid(block_width),
                    tick.pos.y,
                    tick.pos.z.rem_euclid(block_width),
                );
                TickEntry { pos, ..tick.into() }
            })
            .collect();
        (volume, ticks)
    }

    pub fn save_to_file(&self, path: impl AsRef<Path>) -> Result<(), PlotSaveError> {
        let path = path.as_ref();
        if let Ok(mut file) = File::open(path)
            && let Ok(version) = read_version(&mut file)
            && version < VERSION
        {
            let mut original = path.as_os_str().to_owned();
            original.push(format!(".v{version}"));
            fs::copy(path, original)?;
        }
        replace_file(path, |file| {
            let mut file = BufWriter::with_capacity(WRITE_BUFFER_SIZE, file);
            file.write_all(PLOT_MAGIC)?;
            file.write_u32::<LittleEndian>(VERSION)?;
            bincode::serialize_into(&mut file, self)?;
            Ok(file.flush()?)
        })
    }
}

fn chunk_width(chunk_count: usize) -> i32 {
    chunk_count.isqrt() as i32
}

fn read_version(file: &mut File) -> Result<u32, PlotLoadError> {
    let mut magic = [0; 8];
    file.read_exact(&mut magic)?;
    if &magic != PLOT_MAGIC {
        return Err(PlotLoadError::InvalidHeader);
    }
    Ok(file.read_u32::<LittleEndian>()?)
}
