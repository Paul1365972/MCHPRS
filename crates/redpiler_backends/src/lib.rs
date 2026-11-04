use mchprs_commands::flag::{self, FlagValues};
use mchprs_commands::reader::Reader;
use mchprs_redpiler::backend::{Backend, CompilePlan};
use mchprs_redpiler_direct::DirectBackend;
use std::fmt;

pub static BACKENDS: &[&dyn Backend] = &[&DirectBackend];

pub fn named(name: &str) -> Result<&'static dyn Backend, String> {
    BACKENDS
        .iter()
        .copied()
        .find(|backend| backend.name() == name)
        .ok_or_else(|| {
            let names: Vec<_> = BACKENDS.iter().map(|backend| backend.name()).collect();
            format!(
                "unknown backend `{name}` (expected one of: {})",
                names.join(", ")
            )
        })
}

#[derive(Clone)]
pub struct BackendSetup {
    backend: &'static dyn Backend,
    flags: FlagValues,
}

impl BackendSetup {
    pub fn new(backend: &'static dyn Backend, flags: &str) -> Result<BackendSetup, String> {
        let flags = flag::parse(&mut Reader::new(flags), backend.flags())
            .map_err(|error| error.contextual(flags))?;
        Ok(BackendSetup { backend, flags })
    }

    pub fn parse(setup: &str) -> Result<BackendSetup, String> {
        let setup = setup.trim();
        let (name, flags) = setup.split_once(char::is_whitespace).unwrap_or((setup, ""));
        BackendSetup::new(named(name)?, flags)
    }

    pub fn backend(&self) -> &'static dyn Backend {
        self.backend
    }

    pub fn flags(&self) -> &FlagValues {
        &self.flags
    }

    pub fn overridden(&self, flags: FlagValues) -> BackendSetup {
        let mut overridden = self.clone();
        overridden.flags.extend(flags);
        overridden
    }

    pub fn configure(&self) -> Result<CompilePlan, String> {
        self.backend.configure(&self.flags)
    }
}

impl Default for BackendSetup {
    fn default() -> Self {
        BackendSetup {
            backend: BACKENDS[0],
            flags: FlagValues::default(),
        }
    }
}

impl fmt::Display for BackendSetup {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let flags = self.flags.to_string();
        if flags.is_empty() {
            write!(f, "{}", self.backend.name())
        } else {
            write!(f, "{} {flags}", self.backend.name())
        }
    }
}
