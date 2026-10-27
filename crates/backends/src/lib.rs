#[cfg(feature = "vanilla")]
pub mod vanilla;

use mchprs_blocks::block_volume::BlockVolume;
#[cfg(feature = "graph")]
use mchprs_redpiler::backend::GraphBackend;
use mchprs_simulation::backend::{Backend, CompileContext, CompileError, Source};
use mchprs_simulation::layout::Layout;
use mchprs_simulation::simulated_world::SimulatedWorld;
use mchprs_simulation::{Blocks, Simulation};
use mchprs_syntax::flag::{Flag, FlagValues, InvalidFlags};
use std::fmt;

#[cfg(feature = "graph")]
pub const GRAPH_BACKENDS: &[&dyn GraphBackend] = &[
    #[cfg(feature = "direct")]
    &mchprs_redpiler_direct::backend::DirectBackend,
    #[cfg(feature = "phase")]
    &mchprs_redpiler_phase::backend::PhaseBackend,
];

#[cfg(feature = "graph")]
pub fn backends() -> Vec<&'static dyn Backend> {
    GRAPH_BACKENDS
        .iter()
        .map(|&backend| backend as &dyn Backend)
        .collect()
}

#[cfg(not(feature = "graph"))]
pub fn backends() -> Vec<&'static dyn Backend> {
    Vec::new()
}

#[derive(Debug, thiserror::Error)]
pub enum ParseError {
    #[error("unknown backend `{name}` (expected one of: {})", .known.join(", "))]
    UnknownBackend { name: String, known: Vec<String> },
    #[error(transparent)]
    Flags(#[from] InvalidFlags),
}

#[derive(Clone)]
pub struct Setup {
    backend: &'static dyn Backend,
    flags: FlagValues,
}

impl Setup {
    pub fn new(backend: &'static dyn Backend, flags: FlagValues) -> Setup {
        Setup { backend, flags }
    }

    pub fn parse(text: &str) -> Result<Setup, ParseError> {
        let (setup, _) = Setup::parse_among(text, &backends(), &[])?;
        Ok(setup)
    }

    pub fn parse_among(
        text: &str,
        backends: &[&'static dyn Backend],
        host_flags: &[Flag],
    ) -> Result<(Setup, FlagValues), ParseError> {
        let text = text.trim();
        let (name, flags) = text.split_once(char::is_whitespace).unwrap_or((text, ""));
        let named = backends
            .iter()
            .copied()
            .find(|backend| backend.name() == name);
        let backend = named.ok_or_else(|| ParseError::UnknownBackend {
            name: name.to_owned(),
            known: backends
                .iter()
                .map(|backend| backend.name().to_owned())
                .collect(),
        })?;
        let accepted = [backend.flags(), host_flags].concat();
        let mut flags = FlagValues::parse(flags, &accepted)?;
        let host_values = flags.take(host_flags);
        Ok((Setup::new(backend, flags), host_values))
    }

    pub fn name(&self) -> &str {
        self.backend.name()
    }

    pub fn accepted_flags(&self) -> &[Flag] {
        self.backend.flags()
    }

    pub fn flags(&self) -> &FlagValues {
        &self.flags
    }

    pub fn with_flags(&self, flags: FlagValues) -> Setup {
        let mut merged = self.flags.clone();
        merged.extend(flags);
        Setup::new(self.backend, merged)
    }

    pub fn compile(
        &self,
        source: &Source<'_>,
        context: &mut CompileContext<'_>,
    ) -> Result<(Box<dyn Simulation<Blocks>>, Layout), CompileError> {
        self.backend.compile(source, &self.flags, context)
    }

    pub fn simulate(&self, source: &Source<'_>) -> Result<SimulatedWorld, CompileError> {
        let (simulation, layout) = self.compile(source, &mut CompileContext::silent())?;
        let world = BlockVolume::copy(&source.blocks, source.region);
        Ok(SimulatedWorld::new(world, simulation, layout))
    }
}

impl fmt::Display for Setup {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let flags = self.flags.to_string();
        if flags.is_empty() {
            write!(f, "{}", self.name())
        } else {
            write!(f, "{} {flags}", self.name())
        }
    }
}
