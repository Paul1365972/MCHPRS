use petgraph::stable_graph::StableDiGraph;
use serde::{Deserialize};
use std::collections::HashMap;

#[derive(Default)]
struct GraphModule {
    graph: StableDiGraph::<Node, ()>,
}

enum Node {
    NOT,
}



impl YosysJson {
    fn into_graph(&self) {
        let mut graphmodule = GraphModule::default();
        let module = self.modules.values().next().unwrap();
        module.cells.iter().for_each(|(name, cell)| {
            graphmodule.graph.add_node(weight)
        });
    }
}

#[derive(Deserialize)]
pub struct YosysJson {
    modules: HashMap<String, Module>,
}

type Bits = Vec<u32>;

#[derive(Deserialize)]
struct Module {
    ports: HashMap<String, Port>,
    cells: HashMap<String, Cell>,
}

#[derive(Deserialize)]
enum Direction {
    INPUT,
    OUTPUT,
    INOUT,
}

#[derive(Deserialize)]
struct Port {
    direction: Direction,
    bits: Bits,
}

#[derive(Deserialize)]
struct Cell {
    #[serde(rename = "type")]
    kind: String,
    #[serde(rename = "bits")]
    connections: HashMap<String, Bits>,
}
