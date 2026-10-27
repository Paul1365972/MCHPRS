use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::extract::ExtractOptions;
use mchprs_redpiler::graph::{Graph, NodeIdx};
use mchprs_redpiler_phase::engine::PhaseEngine;
use mchprs_ril::Module;
use mchprs_simulation::Simulation;

pub struct Run {
    pub circuit: Circuit,
    pub engine: PhaseEngine,
}

impl Run {
    pub fn new(graph: Graph) -> Run {
        let circuit = Circuit::new(graph).expect("the circuit is valid");
        let engine = PhaseEngine::new(&circuit);
        Run { circuit, engine }
    }

    pub fn node(&self, name: &str) -> NodeId {
        let mut nodes = self.circuit.ids();
        nodes
            .find(|&node| self.circuit.node(node).names == [name])
            .expect("the circuit names the node")
    }

    pub fn tick(&mut self) {
        self.engine.run(1).expect("the engine runs");
    }
}

pub fn graph(ril: &str) -> Graph {
    let module = Module::parse(ril).expect("the circuit parses");
    let mut graph = module.circuits()[0].graph();
    ExtractOptions::default().observe(&mut graph);
    graph
}

pub fn node_named(graph: &Graph, name: &str) -> NodeIdx {
    let mut nodes = graph.node_indices();
    nodes
        .find(|&node| graph[node].names == [name])
        .expect("the circuit names the node")
}
