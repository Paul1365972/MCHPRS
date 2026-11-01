//! This implements Sponge Schematic Specification versions 2 and 3 for reading, and version 2 for writing.
//! https://github.com/SpongePowered/Schematic-Specification/tree/master/versions

mod read;
mod write;

use mchprs_blocks::block_volume::BlockVolume;
use std::collections::BTreeMap;
use thiserror::Error;

pub use read::read;
pub use write::write;

#[derive(Debug)]
pub struct Schematic {
    pub volume: BlockVolume,
    pub unknown_blocks: BTreeMap<String, usize>,
}

#[derive(Debug, Error)]
pub enum SchematicError {
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error(transparent)]
    Nbt(#[from] fastnbt::error::Error),
    #[error("unsupported schematic version {0}")]
    UnsupportedVersion(i32),
    #[error("malformed schematic: {0}")]
    Malformed(String),
    #[error("a schematic must be 1 to 65535 blocks wide, tall and long")]
    UnsupportedSize,
}
