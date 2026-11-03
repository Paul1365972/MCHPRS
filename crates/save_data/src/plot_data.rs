mod fixer;
mod v2_to_v3;
mod v3_to_v4;

use self::fixer::FixInfo;
use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::BlockPos;
use mchprs_redstone::tick_queue::TickPriority;
use mchprs_world::storage::{Chunk, ChunkSection};
use rustc_hash::FxHashMap;
use serde::{Deserialize, Serialize};
use std::borrow::Cow;
use std::fs::{self, File};
use std::io::{BufWriter, IntoInnerError, Read, Write};
use std::path::Path;
use std::{fmt, io};
use thiserror::Error;

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

    #[error("plot data version {0} failed to be converted")]
    ConversionFailed(u32),

    #[error(transparent)]
    Io(#[from] io::Error),

    #[error("conversion from plot data version {0} is unavailable")]
    ConversionUnavailable(u32),
}

impl From<PlotSaveError> for PlotLoadError {
    fn from(e: PlotSaveError) -> Self {
        match e {
            PlotSaveError::Serialize(err) => err.into(),
            PlotSaveError::Io(err) => err.into(),
        }
    }
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
    /// Takes a mutable Chunk to flush it first
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
pub struct PlotData<'a> {
    pub tps: Tps,
    pub world_send_rate: WorldSendRate,
    pub chunk_data: Vec<ChunkData<'a>>,
    pub pending_ticks: Vec<SavedTick>,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SavedTick {
    pub ticks_left: u32,
    pub priority: TickPriority,
    pub position: BlockPos,
    pub block_id: u32,
}

impl PlotData<'static> {
    pub fn load_from_file(path: impl AsRef<Path>) -> Result<Self, PlotLoadError> {
        let mut file = File::open(&path)?;

        let mut magic = [0; 8];
        file.read_exact(&mut magic)?;
        if &magic != PLOT_MAGIC {
            return fixer::try_fix(path, FixInfo::InvalidHeader)?
                .ok_or(PlotLoadError::InvalidHeader);
        }

        let version = file.read_u32::<LittleEndian>()?;
        if version < VERSION {
            return fixer::try_fix(path, FixInfo::OldVersion { version })?
                .ok_or(PlotLoadError::ConversionFailed(version));
        }
        if version > VERSION {
            return Err(PlotLoadError::TooNew(version));
        }

        let mut buf = Vec::new();
        file.read_to_end(&mut buf)?;
        Ok(bincode::deserialize(&buf)?)
    }
}

impl PlotData<'_> {
    pub fn save_to_file(&self, path: impl AsRef<Path>) -> Result<(), PlotSaveError> {
        let path = path.as_ref();
        let mut unfinished = path.as_os_str().to_owned();
        unfinished.push(".tmp");
        let mut file = BufWriter::with_capacity(WRITE_BUFFER_SIZE, File::create(&unfinished)?);

        file.write_all(PLOT_MAGIC)?;
        file.write_u32::<LittleEndian>(VERSION)?;
        bincode::serialize_into(&mut file, self)?;
        let file = file.into_inner().map_err(IntoInnerError::into_error)?;
        file.sync_data()?;
        fs::rename(unfinished, path)?;
        Ok(())
    }
}
