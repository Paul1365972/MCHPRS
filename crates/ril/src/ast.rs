use mchprs_blocks::BlockPos;
use mchprs_redpiler::graph::{Graph, Link, Node, NodeIdx, NodeKind, NodeState, QueuedTick};
use mchprs_simulation::ports::Role;
use std::ops::Range;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Circuit {
    pub name: String,
    pub components: Vec<Component>,
    pub span: Range<usize>,
}

impl Circuit {
    /// Node indices follow the components. No node is observed.
    pub fn graph(&self) -> Graph {
        let mut graph = Graph::default();
        for component in &self.components {
            graph.add_node(Node {
                kind: component.kind,
                state: component.state,
                names: component.names().cloned().collect(),
                blocks: component.blocks.iter().copied().collect(),
                pending_tick: component.pending_tick,
                port: component.port,
                observed: false,
            });
        }
        for (index, component) in self.components.iter().enumerate() {
            for output in &component.outputs {
                graph.add_link(
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
    pub labels: Vec<String>,
    pub kind: NodeKind,
    pub state: NodeState,
    pub port: Option<Role>,
    pub pending_tick: Option<QueuedTick>,
    pub blocks: Vec<(BlockPos, u32)>,
    /// In the order the node updates them
    pub outputs: Vec<Output>,
}

impl Component {
    pub fn names(&self) -> impl Iterator<Item = &String> {
        let is_name = |label: &&String| !label.bytes().all(|byte| byte.is_ascii_digit());
        self.labels.iter().filter(is_name)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Output {
    pub target: usize,
    pub link: Link,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Schematic {
    pub name: String,
    /// Relative to the RIL file
    pub path: String,
}
