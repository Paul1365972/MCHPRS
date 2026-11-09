use crate::input::{self, World};
use crate::oracle::reference::ReferenceBackend;
use crate::Error;
use anyhow::{anyhow, bail};
use mchprs_backends::vanilla::VanillaBackend;
use mchprs_backends::{Setup as BackendSetup, GRAPH_BACKENDS};
use mchprs_redpiler::backend::{GraphBackend, GraphContext};
use mchprs_redpiler::circuit::Circuit;
use mchprs_redpiler::graph::Graph;
use mchprs_redpiler::graph_simulation;
use mchprs_redpiler::nodes::Nodes;
use mchprs_redpiler::ports::WireOutputs;
use mchprs_ril::passes::{self, PASSES};
use mchprs_simulation::backend::{Backend, CompileContext};
use mchprs_simulation::layout::Layout;
use mchprs_simulation::simulated_world::SimulatedWorld;
use mchprs_simulation::{Blocks, Simulation};
use mchprs_syntax::flag::FlagValues;
use std::fmt;

#[derive(Clone)]
pub struct Setup {
    backend: BackendSetup,
    graph: Option<&'static dyn GraphBackend>,
    passes: Option<String>,
}

impl Setup {
    pub fn parse(text: &str) -> Result<Setup, Error> {
        let (backend, own_values) = BackendSetup::parse_among(text, &backends(), &[PASSES])?;
        let graph = find_graph_backend(backend.name());
        let passes = match (own_values.text(&PASSES), &graph) {
            (None, _) => None,
            (Some(_), None) => {
                let (name, passes) = (backend.name(), PASSES.name());
                bail!(
                    "the {name} backend doesn't run on the shared graph, so it takes no --{passes}"
                );
            }
            (Some(names), Some(_)) => {
                passes::named(names)?;
                Some(names.to_owned())
            }
        };
        Ok(Setup {
            backend,
            graph,
            passes,
        })
    }

    pub fn reference() -> Setup {
        Setup {
            backend: BackendSetup::new(&ReferenceBackend, FlagValues::default()),
            graph: Some(&ReferenceBackend),
            passes: None,
        }
    }

    pub fn flags(&self) -> &FlagValues {
        self.backend.flags()
    }

    pub fn compile(&self, world: &World) -> Result<(Box<dyn Simulation<Blocks>>, Layout), Error> {
        let source = input::source(world);
        if self.passes.is_none() {
            return Ok(self
                .backend
                .compile(&source, &mut CompileContext::silent())?);
        }
        let context = &mut GraphContext::silent();
        let graph = self.world_graph(world, context)?;
        let backend = self.graph_backend()?;
        Ok(graph_simulation::build(
            backend,
            graph,
            &source,
            self.flags(),
            context,
        )?)
    }

    pub fn simulate(&self, world: &World) -> Result<SimulatedWorld, Error> {
        let (simulation, layout) = self.compile(world)?;
        Ok(SimulatedWorld::new(
            world.blocks().clone(),
            simulation,
            layout,
        ))
    }

    pub fn world_graph(
        &self,
        world: &World,
        context: &mut GraphContext<'_>,
    ) -> Result<Graph, Error> {
        let backend = self.graph_backend()?;
        let source = input::source(world);
        let mut graph = graph_simulation::extract(backend, &source, self.flags(), context)?;
        self.optimize(backend, &mut graph, context)?;
        Ok(graph)
    }

    pub fn circuit_graph(
        &self,
        mut graph: Graph,
        context: &mut GraphContext<'_>,
    ) -> Result<Graph, Error> {
        let backend = self.circuit_backend()?;
        backend.extract_options(self.flags()).observe(&mut graph);
        context.inspect("parsed", &graph);
        self.optimize(backend, &mut graph, context)?;
        Ok(graph)
    }

    pub fn build(&self, graph: Graph) -> Result<(Circuit, Box<dyn Simulation<Nodes>>), Error> {
        let circuit = Circuit::new(self.circuit_graph(graph, &mut GraphContext::silent())?)?;
        let simulation = self.build_circuit(&circuit)?;
        Ok((circuit, simulation))
    }

    pub fn build_circuit(&self, circuit: &Circuit) -> Result<Box<dyn Simulation<Nodes>>, Error> {
        let backend = self.circuit_backend()?;
        Ok(backend.build(circuit, self.flags(), &mut GraphContext::silent())?)
    }

    fn circuit_backend(&self) -> Result<&dyn GraphBackend, Error> {
        let backend = self.graph_backend()?;
        let options = backend.extract_options(self.flags());
        if options.wire_outputs != WireOutputs::default() {
            let setup = &self.backend;
            bail!("{setup} treats wires as outputs, which only applies to worlds");
        }
        Ok(backend)
    }

    pub fn graph_backend(&self) -> Result<&dyn GraphBackend, Error> {
        let name = self.backend.name();
        let backend = self.graph;
        backend.ok_or_else(|| anyhow!("the {name} backend doesn't run on the shared graph"))
    }

    fn optimize(
        &self,
        backend: &dyn GraphBackend,
        graph: &mut Graph,
        context: &mut GraphContext<'_>,
    ) -> Result<(), Error> {
        match &self.passes {
            Some(names) => context.run_passes(graph, &passes::named(names)?)?,
            None => backend.optimize(graph, self.flags(), context)?,
        }
        Ok(())
    }
}

impl Default for Setup {
    fn default() -> Setup {
        match GRAPH_BACKENDS.first() {
            Some(&backend) => Setup {
                backend: BackendSetup::new(backend, FlagValues::default()),
                graph: Some(backend),
                passes: None,
            },
            None => Setup::reference(),
        }
    }
}

impl fmt::Display for Setup {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.backend)?;
        if let Some(names) = &self.passes {
            write!(f, " --{} {names}", PASSES.name())?;
        }
        Ok(())
    }
}

pub fn help() -> String {
    let backends = backends();
    let width = backends
        .iter()
        .flat_map(|backend| backend.flags())
        .map(|flag| flag.to_string().len())
        .max()
        .unwrap_or(0);
    let mut help = "Backends and their flags:".to_owned();
    for backend in &backends {
        help.push_str(&format!("\n  {}", backend.name()));
        for flag in backend.flags() {
            let usage = flag.to_string();
            help.push_str(&format!("\n    {usage:width$}  {}", flag.description()));
        }
    }
    let passes = PASSES.name();
    let description = PASSES.description();
    help.push_str(&format!(
        "\nA setup of a graph backend also takes --{passes}=PASSES: {description}"
    ));
    help
}

fn backends() -> Vec<&'static dyn Backend> {
    mchprs_backends::backends()
        .into_iter()
        .chain([&VanillaBackend as &dyn Backend, &ReferenceBackend])
        .collect()
}

fn find_graph_backend(name: &str) -> Option<&'static dyn GraphBackend> {
    GRAPH_BACKENDS
        .iter()
        .copied()
        .chain([&ReferenceBackend as &dyn GraphBackend])
        .find(|graph| graph.name() == name)
}
