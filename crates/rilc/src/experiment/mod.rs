mod diff;

use crate::read_schematic_at_origin;
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_commands::flag::{self, FlagValues};
use mchprs_commands::reader::Reader;
use mchprs_redpiler::backend::{Backend, CompilePlan};
use mchprs_redpiler::circuit::{Circuit, CircuitFormatError, NodeId};
use mchprs_redpiler::compile::compile;
use mchprs_redpiler::engine::Engine;
use mchprs_redpiler::ports::Role;
use mchprs_redpiler_backends::BACKENDS;
use std::error::Error;
use std::fmt;
use std::fs;
use std::ops::ControlFlow;
use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

pub use diff::diff;

type BoxError = Box<dyn Error + Send + Sync>;

#[derive(Clone)]
pub struct EngineSpec {
    backend: &'static dyn Backend,
    flags: FlagValues,
}

impl EngineSpec {
    pub fn parse(spec: &str) -> Result<EngineSpec, String> {
        let spec = spec.trim();
        let (name, flags) = spec.split_once(char::is_whitespace).unwrap_or((spec, ""));
        let backend = mchprs_redpiler_backends::named(name)?;
        let flags = flag::parse(&mut Reader::new(flags), backend.flags())
            .map_err(|error| error.contextual(flags))?;
        Ok(EngineSpec { backend, flags })
    }

    pub fn configure(&self) -> Result<CompilePlan, String> {
        self.backend.configure(&self.flags)
    }
}

impl Default for EngineSpec {
    fn default() -> Self {
        EngineSpec {
            backend: BACKENDS[0],
            flags: FlagValues::default(),
        }
    }
}

impl fmt::Display for EngineSpec {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let flags = self.flags.to_string();
        if flags.is_empty() {
            write!(f, "{}", self.backend.name())
        } else {
            write!(f, "{} {flags}", self.backend.name())
        }
    }
}

pub struct Configured {
    label: String,
    plan: CompilePlan,
}

impl Configured {
    pub fn new(spec: EngineSpec) -> Result<Configured, String> {
        Ok(Configured {
            label: spec.to_string(),
            plan: spec.configure()?,
        })
    }

    fn start(self, source: &Source) -> Result<Box<dyn Engine>, BoxError> {
        let circuit = source.circuit(&self.plan)?;
        (self.plan.build)(circuit)
    }
}

pub enum Source {
    Circuit(Arc<Circuit>),
    Schematic(BlockVolume),
}

impl Source {
    pub fn read(path: &Path) -> Result<Source, BoxError> {
        let bytes = fs::read(path)
            .map_err(|error| format!("couldn't read `{}`: {error}", path.display()))?;
        match Circuit::from_bytes(&bytes) {
            Ok(circuit) => Ok(Source::Circuit(Arc::new(circuit))),
            Err(CircuitFormatError::NotACircuit) => {
                Ok(Source::Schematic(read_schematic_at_origin(&bytes)?))
            }
            Err(error) => Err(format!("`{}`: {error}", path.display()).into()),
        }
    }

    fn circuit(&self, plan: &CompilePlan) -> Result<Arc<Circuit>, BoxError> {
        match self {
            Source::Circuit(circuit) => Ok(circuit.clone()),
            Source::Schematic(volume) => {
                let circuit = compile(
                    volume,
                    volume.region(),
                    &[],
                    &plan.extract,
                    &plan.passes,
                    &mut |_| ControlFlow::Continue(()),
                )?;
                Ok(Arc::new(circuit))
            }
        }
    }
}

pub fn run(source: &Source, engine: Configured, ticks: u64) -> Result<(), BoxError> {
    let name = engine.label.clone();
    let mut engine = engine.start(source)?;
    let circuit = engine.circuit().clone();
    let start = Instant::now();
    engine.run(ticks)?;
    let elapsed = start.elapsed();
    println!(
        "{name} ran {ticks} ticks of {} nodes in {elapsed:?} ({:.0} ticks/s)",
        circuit.len(),
        ticks as f64 / elapsed.as_secs_f64()
    );
    let snapshot = engine.snapshot()?;
    let outputs: Vec<NodeId> = circuit
        .ports()
        .filter(|(_, role)| *role == Role::Output)
        .map(|(node, _)| node)
        .collect();
    let powered = outputs
        .iter()
        .filter(|node| !snapshot.states[node.index()].power.is_zero())
        .count();
    println!(
        "{powered} of {} outputs powered, {} ticks pending",
        outputs.len(),
        snapshot.pending_ticks.len()
    );
    Ok(())
}
