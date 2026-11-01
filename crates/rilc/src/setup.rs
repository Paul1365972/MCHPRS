use mchprs_commands::flag::{self, Flag, FlagValues};
use mchprs_commands::reader::Reader;
use mchprs_redpiler::backend::{BuildError, CompilePlan};
use mchprs_redpiler::circuit::Circuit;
use mchprs_redpiler::compile::BuildContext;
use mchprs_redpiler::engine::Engine;
use mchprs_redpiler::extract::WIRE_PORT_FLAGS;
use mchprs_redpiler::passes::{self, Pass};
use mchprs_redpiler_backends::{BackendSetup, BACKENDS};
use std::fmt;

const PASSES: Flag = Flag::valued(
    "passes",
    "PASSES",
    "Comma-separated passes to run instead of the backend's",
);

#[derive(Clone, Default)]
pub struct Setup {
    backend: BackendSetup,
    passes: Option<Vec<&'static dyn Pass>>,
}

impl Setup {
    pub fn parse(text: &str) -> Result<Setup, String> {
        let text = text.trim();
        let (name, arguments) = text.split_once(char::is_whitespace).unwrap_or((text, ""));
        let backend = mchprs_redpiler_backends::named(name)?;
        let accepted: Vec<Flag> = backend.flags().iter().copied().chain([PASSES]).collect();
        let values = flag::parse(&mut Reader::new(arguments), &accepted)
            .map_err(|error| error.contextual(arguments))?;
        let passes = match values.text(&PASSES) {
            Some(names) => Some(
                names
                    .split(',')
                    .map(|name| passes::named(name).ok_or_else(|| format!("unknown pass {name}")))
                    .collect::<Result<_, _>>()?,
            ),
            None => None,
        };
        let mut backend_flags = FlagValues::default();
        for flag in backend.flags() {
            if let Some(value) = values.get(flag.name()) {
                backend_flags.set(*flag, value.clone());
            }
        }
        Ok(Setup {
            backend: BackendSetup::new(backend, "")?.overridden(backend_flags),
            passes,
        })
    }

    pub fn plan(&self) -> Result<CompilePlan, String> {
        let mut plan = self.backend.configure()?;
        if let Some(passes) = &self.passes {
            plan.passes = passes.clone();
        }
        Ok(plan)
    }

    pub fn check_for_circuit(&self) -> Result<(), String> {
        let flags = self.backend.flags();
        match WIRE_PORT_FLAGS.iter().find(|flag| flags.is_on(flag)) {
            Some(flag) => Err(format!(
                "--{} only applies to worlds, not RIL circuits",
                flag.name()
            )),
            None => Ok(()),
        }
    }

    pub fn build(&self, circuit: &Circuit) -> Result<Box<dyn Engine>, BuildError> {
        self.backend
            .backend()
            .build(circuit, self.backend.flags(), &mut BuildContext::silent())
    }
}

impl fmt::Display for Setup {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.backend)?;
        if let Some(passes) = &self.passes {
            let names: Vec<_> = passes.iter().map(|pass| pass.name()).collect();
            write!(f, " --passes={}", names.join(","))?;
        }
        Ok(())
    }
}

pub fn help() -> String {
    let flags = || {
        BACKENDS
            .iter()
            .flat_map(|backend| backend.flags())
            .chain([&PASSES])
    };
    let width = flags()
        .map(|flag| flag.to_string().len())
        .max()
        .unwrap_or(0);
    let mut help = "Backends and their flags:".to_owned();
    for backend in BACKENDS {
        help.push_str(&format!("\n  {}", backend.name()));
        for flag in backend.flags().iter().chain([&PASSES]) {
            let usage = flag.to_string();
            help.push_str(&format!("\n    {usage:width$}  {}", flag.description()));
        }
    }
    help
}
