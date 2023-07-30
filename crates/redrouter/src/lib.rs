mod yosys_json;

use std::fs;

use mcircuit::parsers::blif::BlifParser;
use petgraph::data::Build;
use yosys_json::YosysJson;

type SimpleGraph = petgraph::stable_graph::StableDiGraph::<SimpleNode, SimpleConnection>;

#[derive(Debug, Default)]
struct SimpleNode;

#[derive(Debug, Default)]
struct SimpleConnection;

fn load_blifs() {
    let json: YosysJson = serde_json::from_str(&fs::read_to_string("./test/counter.json").unwrap()).unwrap();
}

fn example_graph() -> SimpleGraph {
    let mut graph = SimpleGraph::new();
    let a = graph.add_node(Default::default());
    let b = graph.add_node(Default::default());
    let c = graph.add_node(Default::default());
    let d = graph.add_node(Default::default());
    graph.add_edge(a, c, Default::default());
    graph.add_edge(b, c, Default::default());
    graph.add_edge(c, d, Default::default());
    
    return graph;
}

fn initial_placement(graph: SimpleGraph) {
    
}


#[test]
fn test() {
    println!("Starting test");
    let example = example_graph();
    load_blifs();
    println!("Starting test {example:?}");
}
