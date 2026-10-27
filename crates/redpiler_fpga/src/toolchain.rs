use crate::board::{Board, BoardPort};
use crate::process::ToolError;
use crate::progress::Progress;
use crate::rtl::Sources;
use mchprs_simulation::backend::{Cancelled, CompileError};
use std::io;
use std::path::{Path, PathBuf};

/// Builds a bitstream for a board and programs the board with it.
pub trait Toolchain {
    fn detect(
        &self,
        board: &Board,
        port: &BoardPort,
        directory: &Path,
        progress: &Progress,
    ) -> Result<(), ToolchainError>;

    fn build(
        &self,
        sources: &Sources,
        board: &Board,
        progress: &mut Progress,
    ) -> Result<Bitstream, ToolchainError>;

    fn program(
        &self,
        bitstream: &Bitstream,
        board: &Board,
        port: &BoardPort,
        progress: &Progress,
    ) -> Result<(), ToolchainError>;
}

#[derive(Debug, Clone, PartialEq)]
pub struct Bitstream {
    pub path: PathBuf,
    pub fmax_hertz: f64,
    pub luts: u32,
    pub flip_flops: u32,
    pub slices: usize,
}

#[derive(Debug, thiserror::Error)]
pub enum ToolchainError {
    #[error(transparent)]
    Tool(#[from] ToolError),
    #[error("could not access {}", path.display())]
    Io {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("the report {} is not what the tool is known to write", path.display())]
    Report {
        path: PathBuf,
        #[source]
        source: serde_json::Error,
    },
    #[error("the report names no clock {0}")]
    NoClock(&'static str),
    #[error("the clock {clock} of the board reaches {achieved:.1} of its {required:.1} MHz")]
    Timing {
        clock: String,
        achieved: f64,
        required: f64,
    },
    #[error("found no share/{} beside the {tool} on the PATH", data.display())]
    NoData { tool: &'static str, data: PathBuf },
}

impl From<Cancelled> for ToolchainError {
    fn from(cancelled: Cancelled) -> ToolchainError {
        ToolchainError::Tool(cancelled.into())
    }
}

impl From<ToolchainError> for CompileError {
    fn from(error: ToolchainError) -> CompileError {
        match error {
            ToolchainError::Tool(error) => error.into(),
            error => CompileError::Failed(error.into()),
        }
    }
}
