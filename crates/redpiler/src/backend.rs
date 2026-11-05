use crate::circuit::Circuit;
use crate::engine::{Engine, EngineError};
use crate::extract::ExtractOptions;
use crate::passes::Pass;
use mchprs_commands::flag::{Flag, FlagValues};

pub type Build = Box<dyn FnOnce(&Circuit) -> Result<Box<dyn Engine>, EngineError> + Send>;

pub trait Backend: Sync {
    fn name(&self) -> &'static str;

    fn flags(&self) -> &'static [Flag];

    fn configure(&self, flags: &FlagValues) -> Result<CompilePlan, String>;
}

pub struct CompilePlan {
    pub extract: ExtractOptions,
    pub passes: Vec<&'static dyn Pass>,
    pub build: Build,
}
