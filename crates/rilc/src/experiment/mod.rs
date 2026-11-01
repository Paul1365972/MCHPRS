mod diff;

use crate::read_schematic_at_origin;
use clap::ValueEnum;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::circuit::{Circuit, CircuitFormatError, NodeId};
use mchprs_redpiler::compile::compile;
use mchprs_redpiler::engine::{Engine, EngineError, Input, Snapshot};
use mchprs_redpiler::graph::{Drive, PendingTick};
use mchprs_redpiler::ports::Role;
use mchprs_redpiler_direct::DirectEngine;
use mchprs_ril::RilOptions;
use std::error::Error;
use std::fmt;
use std::fs;
use std::num::NonZeroU64;
use std::ops::ControlFlow;
use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

pub use diff::diff;

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum EngineKind {
    Direct,
}

impl EngineKind {
    fn build(self, circuit: Arc<Circuit>) -> Result<Box<dyn Engine>, EngineError> {
        Ok(match self {
            EngineKind::Direct => Box::new(DirectEngine::new(circuit)?),
        })
    }
}

impl fmt::Display for EngineKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            EngineKind::Direct => write!(f, "direct"),
        }
    }
}

/// Reads a circuit file, or compiles a schematic.
pub fn load_circuit(
    path: &Path,
    options: &RilOptions,
) -> Result<Circuit, Box<dyn Error + Send + Sync>> {
    let bytes =
        fs::read(path).map_err(|error| format!("couldn't read `{}`: {error}", path.display()))?;
    match Circuit::from_bytes(&bytes) {
        Ok(circuit) => {
            if *options != RilOptions::default() {
                eprintln!("warning: compile flags don't apply to a compiled circuit");
            }
            return Ok(circuit);
        }
        Err(CircuitFormatError::NotACircuit) => {}
        Err(error) => return Err(format!("`{}`: {error}", path.display()).into()),
    }
    let volume = read_schematic_at_origin(&bytes)?;
    let circuit = compile(
        &volume,
        volume.region(),
        &[],
        &options.compile_options(),
        &mut |_| ControlFlow::Continue(()),
    )?;
    Ok(circuit)
}

pub fn run(
    circuit: Circuit,
    engine_kind: EngineKind,
    ticks: u64,
) -> Result<(), Box<dyn Error + Send + Sync>> {
    let circuit = Arc::new(circuit);
    let mut engine = engine_kind.build(circuit.clone())?;
    let start = Instant::now();
    engine.run(ticks)?;
    let elapsed = start.elapsed();
    println!(
        "{engine_kind} ran {ticks} ticks of {} nodes in {elapsed:?} ({:.0} ticks/s)",
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
