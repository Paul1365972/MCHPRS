use mchprs_redpiler::graph::{Direction, Graph, LinkKind};
use mchprs_ril::{component, NodeNames};
use mchprs_simulation::ports::Role;
use std::fmt;

pub struct Dot<'a> {
    graph: &'a Graph,
    name: &'a str,
}

impl<'a> Dot<'a> {
    pub fn new(graph: &'a Graph, name: &'a str) -> Dot<'a> {
        Dot { graph, name }
    }
}

impl fmt::Display for Dot<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "digraph \"{}\" {{", self.name)?;
        let names = NodeNames::new(self.graph);
        for idx in self.graph.node_indices() {
            let node = &self.graph[idx];
            let component = component(node.kind, node.state);
            write!(
                f,
                "  n{} [label=\"%{}\\n{component}",
                idx.index(),
                names.of(idx)
            )?;
            match node.port {
                Some(Role::Input) => write!(f, "\\ninput")?,
                Some(Role::Output) => write!(f, "\\noutput")?,
                None => {}
            }
            if let Some(queued) = node.pending_tick {
                write!(f, "\\npending {}", queued.tick)?;
            }
            for (pos, _) in &node.blocks {
                write!(f, "\\n{pos}")?;
            }
            writeln!(f, "\"];")?;
            for edge in self.graph.edges(idx, Direction::Outgoing) {
                let link = edge.weight();
                write!(
                    f,
                    "  n{} -> n{} [label=\"{}\"",
                    idx.index(),
                    edge.target().index(),
                    link.weight
                )?;
                if link.kind == LinkKind::Side {
                    write!(f, ", color=blue, fontcolor=blue")?;
                }
                writeln!(f, "];")?;
            }
        }
        write!(f, "}}")
    }
}
