use crate::circuit::Circuit;
use crate::engine::{Engine, EngineError};
use crate::extract::ExtractOptions;
use crate::passes::Pass;
use mchprs_commands::flag::{Flag, FlagValues};

pub trait Backend: Sync {
    fn name(&self) -> &'static str;

    fn flags(&self) -> &'static [Flag];

    fn configure(&self, flags: &FlagValues) -> Result<CompilePlan, String>;

    fn build(&self, circuit: &Circuit, flags: &FlagValues) -> Result<Box<dyn Engine>, EngineError>;
}

pub struct CompilePlan {
    pub extract: ExtractOptions,
    pub passes: Vec<&'static dyn Pass>,
}
