use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::graph::{Direction, Graph, LinkKind, NodeIdx, NodeKind, NodeState};
use mchprs_redpiler::ports::Role;
use rustc_hash::FxHashSet;
use std::fmt;

/// Each node's name in RIL: its first name, or else its index, extended until no named node has it
pub(crate) struct NodeNames(Vec<String>);

impl NodeNames {
    pub(crate) fn new(graph: &Graph) -> NodeNames {
        let taken: FxHashSet<&str> = graph
            .node_indices()
            .filter_map(|node| graph[node].names.first())
            .map(String::as_str)
            .collect();
        let mut names = vec![String::new(); graph.node_bound()];
        for node in graph.node_indices() {
            names[node.index()] = match graph[node].names.first() {
                Some(name) => name.clone(),
                None => {
                    let mut name = node.index().to_string();
                    while taken.contains(name.as_str()) {
                        name.push('_');
                    }
                    name
                }
            };
        }
        NodeNames(names)
    }

    pub(crate) fn of(&self, node: NodeIdx) -> &str {
        &self.0[node.index()]
    }
}

pub struct Printed<'a> {
    graph: &'a Graph,
    name: &'a str,
}

impl<'a> Printed<'a> {
    pub fn new(graph: &'a Graph, name: &'a str) -> Printed<'a> {
        Printed { graph, name }
    }
}

impl fmt::Display for Printed<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "circuit @{} {{", self.name)?;
        let names = NodeNames::new(self.graph);
        for node in self.graph.node_indices() {
            write!(f, "  ")?;
            write_node(f, self.graph, &names, node)?;
            let blocks = &self.graph[node].blocks;
            if !blocks.is_empty() {
                write!(f, "  # Loc: ")?;
                for (index, (pos, _)) in blocks.iter().enumerate() {
                    if index != 0 {
                        write!(f, ", ")?;
                    }
                    write!(f, "{pos}")?;
                }
            }
            writeln!(f)?;
        }
        write!(f, "}}")
    }
}

pub fn component(kind: &NodeKind, state: NodeState) -> impl fmt::Display + '_ {
    Component { kind, state }
}

struct Component<'a> {
    kind: &'a NodeKind,
    state: NodeState,
}

impl fmt::Display for Component<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let state = self.state;
        let powered = state.is_powered();
        match *self.kind {
            NodeKind::Repeater { delay, prioritized } => write!(
                f,
                "repeater {delay}, {prioritized}, {}, {powered}",
                state.repeater_locked
            ),
            NodeKind::Torch => write!(f, "torch {powered}"),
            NodeKind::Comparator {
                mode,
                far_input,
                prioritized,
            } => write!(
                f,
                "comparator {mode}, {}, {prioritized}, {}",
                FarInput(far_input),
                state.power
            ),
            NodeKind::Lamp => write!(f, "lamp {powered}"),
            NodeKind::Button { press_ticks } => write!(f, "button {press_ticks}, {powered}"),
            NodeKind::Lever => write!(f, "lever {powered}"),
            NodeKind::PressurePlate => write!(f, "pressure_plate {powered}"),
            NodeKind::Trapdoor => write!(f, "trapdoor {powered}"),
            NodeKind::Wire => write!(f, "wire {}", state.power),
            NodeKind::Constant if state.repeater_locked => {
                write!(f, "constant {}, true", state.power)
            }
            NodeKind::Constant => write!(f, "constant {}", state.power),
            NodeKind::NoteBlock { instrument, note } => {
                write!(f, "note_block {instrument}, {note}, {powered}")
            }
        }
    }
}

struct FarInput(Option<SignalStrength>);

impl fmt::Display for FarInput {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            Some(power) => write!(f, "{power}"),
            None => write!(f, "none"),
        }
    }
}

pub(crate) fn write_node(
    f: &mut impl fmt::Write,
    graph: &Graph,
    names: &NodeNames,
    idx: NodeIdx,
) -> fmt::Result {
    let node = &graph[idx];
    write!(f, "%{}", names.of(idx))?;
    write!(f, " = {}", component(&node.kind, node.state))?;
    if node.port != node.kind.conventional_role() {
        let marker = match node.port {
            Some(Role::Input) => "input",
            Some(Role::Output) => "output",
            None => "internal",
        };
        write!(f, " {marker}")?;
    }
    let mut outputs = graph.edges(idx, Direction::Outgoing).peekable();
    if outputs.peek().is_none() {
        return Ok(());
    }
    write!(f, " -> [")?;
    for (index, edge) in outputs.enumerate() {
        if index > 0 {
            write!(f, ", ")?;
        }
        if edge.weight().kind == LinkKind::Side {
            write!(f, "side ")?;
        }
        write!(f, "%{}:{}", names.of(edge.target()), edge.weight().weight)?;
    }
    write!(f, "]")
}
