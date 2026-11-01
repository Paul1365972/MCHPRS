use mchprs_redpiler::extract::ExtractOptions;
use mchprs_redpiler::graph::{Graph, Link, Node, NodeIdx, NodeKind, NodeState};
use mchprs_redpiler::ports::Role;
use std::ops::Range;

/// A circuit whose names are resolved and which keeps the circuit invariants.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Circuit {
    pub name: String,
    pub components: Vec<Component>,
    /// From the `circuit` keyword through the closing brace
    pub span: Range<usize>,
}

impl Circuit {
    /// Observes the nodes that `options` observes. Node indices follow the components.
    pub fn graph(&self, options: &ExtractOptions) -> Graph {
        self.build_graph(|port| options.observes(port))
    }

    pub(crate) fn build_graph(&self, observes: impl Fn(Option<Role>) -> bool) -> Graph {
        let mut graph = Graph::default();
        for component in &self.components {
            graph.add_node(Node {
                kind: component.kind.clone(),
                state: component.state,
                names: vec![component.name.clone()],
                blocks: Default::default(),
                pending_tick: None,
                port: component.port,
                observed: observes(component.port),
            });
        }
        for (index, component) in self.components.iter().enumerate() {
            for output in &component.outputs {
                graph.add_edge(
                    NodeIdx::new(index),
                    NodeIdx::new(output.target),
                    output.link,
                );
            }
        }
        graph
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Component {
    pub name: String,
    pub kind: NodeKind,
    pub state: NodeState,
    pub port: Option<Role>,
    /// In the order the node updates them
    pub outputs: Vec<Output>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Output {
    /// The index of the target among the circuit's components
    pub target: usize,
    pub link: Link,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Schematic {
    pub name: String,
    /// Relative to the RIL file
    pub path: String,
}
