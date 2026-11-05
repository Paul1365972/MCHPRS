mod diff;

use crate::read_schematic_at_origin;
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_redpiler::backend::CompilePlan;
use mchprs_redpiler::circuit::{Circuit, CircuitFormatError, NodeId};
use mchprs_redpiler::compile::{compile, BuildContext};
use mchprs_redpiler::engine::Engine;
use mchprs_redpiler::ports::Role;
use mchprs_redpiler_backends::BackendSetup;
use std::error::Error;
use std::fs;
use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

pub use diff::diff;

type BoxError = Box<dyn Error + Send + Sync>;

pub struct Configured {
    setup: BackendSetup,
    plan: CompilePlan,
}

impl Configured {
    pub fn new(setup: BackendSetup) -> Result<Configured, String> {
        Ok(Configured {
            plan: setup.configure()?,
            setup,
        })
    }

    fn start(self, source: &Source) -> Result<(Arc<Circuit>, Box<dyn Engine>), BoxError> {
        let circuit = source.circuit(&self.plan)?;
        let engine = self.setup.backend().build(
            &circuit,
            self.setup.flags(),
            &mut BuildContext::silent(),
        )?;
        Ok((circuit, engine))
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
                    &mut BuildContext::silent(),
                )?;
                Ok(Arc::new(circuit))
            }
        }
    }
}

pub fn run(source: &Source, backend: Configured, ticks: u64) -> Result<(), BoxError> {
    let name = backend.setup.to_string();
    let (circuit, mut engine) = backend.start(source)?;
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
