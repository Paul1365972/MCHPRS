use crate::circuit::Circuit;
use crate::compile::{BuildContext, Cancelled};
use crate::engine::{Engine, EngineError};
use crate::extract::ExtractOptions;
use crate::passes::Pass;
use mchprs_commands::flag::{Flag, FlagValues};
use std::fmt;

pub trait Backend: Sync {
    fn name(&self) -> &'static str;

    fn flags(&self) -> &'static [Flag];

    fn configure(&self, flags: &FlagValues) -> Result<CompilePlan, String>;

    fn build(
        &self,
        circuit: &Circuit,
        flags: &FlagValues,
        context: &mut BuildContext<'_>,
    ) -> Result<Box<dyn Engine>, BuildError>;
}

pub struct CompilePlan {
    pub extract: ExtractOptions,
    pub passes: Vec<&'static dyn Pass>,
}

#[derive(Debug)]
pub enum BuildError {
    Cancelled,
    Unsupported(String),
    Failed(EngineError),
}

impl fmt::Display for BuildError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BuildError::Cancelled => write!(f, "build cancelled"),
            BuildError::Unsupported(reason) => reason.fmt(f),
            BuildError::Failed(error) => error.fmt(f),
        }
    }
}

impl std::error::Error for BuildError {}

impl From<Cancelled> for BuildError {
    fn from(_: Cancelled) -> Self {
        BuildError::Cancelled
    }
}
