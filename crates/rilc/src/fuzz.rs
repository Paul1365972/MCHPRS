mod circuit_generator;
mod engine;
mod passes;
mod report;
mod rng;
mod scenario_generator;

use crate::input::Loaded;
use crate::runner::labelled_graph;
use crate::setup::Setup;
use crate::{Error, Outcome};
use circuit_generator::CircuitCase;
use clap::{Args, Subcommand};
use mchprs_redpiler::backend::GraphContext;
use mchprs_redpiler::circuit::Circuit;
use mchprs_redpiler::graph::Graph;
use rng::Rng;
use std::path::PathBuf;

#[derive(Subcommand)]
pub enum Property {
    /// `direct` runs random circuits in an order they may run in
    ///
    /// After every input and every tick, what `direct` shows at every node and the ticks it has
    /// pending are what the circuit shows in some order it may run in, and the changes it reports
    /// bring the observed nodes to the state it has.
    Engine {
        /// Schematics, plot files or RIL files of one circuit to check instead of random circuits
        inputs: Vec<PathBuf>,
        #[command(flatten)]
        cases: Cases,
    },
    /// What a small random circuit shows after passes is what it shows without them in some
    /// order it may run in
    Passes {
        /// Schematics, plot files or RIL files of one circuit to check instead of random circuits
        inputs: Vec<PathBuf>,
        /// Check this comma-separated pass list, instead of each pass alone and each pass of
        /// `direct --optimize --io-only` after its predecessors
        #[arg(long, value_name = "LIST")]
        passes: Option<String>,
        #[command(flatten)]
        cases: Cases,
    },
}

#[derive(Args)]
pub struct Cases {
    /// The seed of the first case; the case after it has the next seed
    #[arg(long, value_name = "S", default_value = "1")]
    seed: u64,
    /// How many cases to run
    #[arg(long = "cases", value_name = "N", default_value = "1000")]
    count: u64,
}

pub fn command(property: Property) -> Outcome {
    match property {
        Property::Engine { inputs, cases } => engine::command(&inputs, &cases),
        Property::Passes {
            inputs,
            passes,
            cases,
        } => passes::command(&inputs, passes.as_deref(), &cases),
    }
}

struct GivenCircuits(Vec<(String, Graph)>);

impl GivenCircuits {
    fn read(inputs: &[PathBuf]) -> Result<GivenCircuits, Error> {
        let reference = Setup::reference();
        let mut given = Vec::new();
        for path in inputs {
            let context = &mut GraphContext::silent();
            let graph = match Loaded::read(path, None)? {
                Loaded::World(world) => reference.world_graph(&world, context)?,
                Loaded::Circuit(circuit) => {
                    reference.circuit_graph(labelled_graph(&circuit), context)?
                }
            };
            Circuit::check(&graph)?;
            given.push((path.display().to_string(), graph));
        }
        Ok(GivenCircuits(given))
    }

    fn random_case(&self, rng: &mut Rng, ticks: u64) -> Option<(&str, CircuitCase)> {
        if self.0.is_empty() {
            return None;
        }
        let (path, graph) = &self.0[rng.below(self.0.len() as u64) as usize];
        Some((path, CircuitCase::new(graph.clone(), rng, ticks)))
    }
}
