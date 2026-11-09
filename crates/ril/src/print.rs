use crate::fields::{kind_name, visit, visit_tick, Fields, Value, PORTS};
use mchprs_blocks::blocks::Block;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::graph::{Direction, Graph, LinkKind, NodeIdx, NodeKind, NodeState};
use mchprs_redstone::tick_queue::PendingTick;
use rustc_hash::FxHashMap;
use std::fmt;

pub struct NodeNames(Vec<String>);

impl NodeNames {
    pub fn new(graph: &Graph) -> NodeNames {
        let mut names = vec![String::new(); graph.node_bound()];
        for (position, node) in graph.node_indices().enumerate() {
            names[node.index()] = match graph[node].names.first() {
                Some(name) => name.clone(),
                None => position.to_string(),
            };
        }
        NodeNames(names)
    }

    pub fn of(&self, node: NodeIdx) -> &str {
        &self.0[node.index()]
    }
}

pub struct TickOrders(FxHashMap<NodeIdx, usize>);

impl TickOrders {
    pub fn new(graph: &Graph) -> TickOrders {
        let mut due_together: FxHashMap<PendingTick, Vec<(usize, NodeIdx)>> = FxHashMap::default();
        for node in graph.node_indices() {
            if let Some(queued) = graph[node].pending_tick {
                due_together
                    .entry(queued.tick)
                    .or_default()
                    .push((queued.order, node));
            }
        }
        let mut orders = FxHashMap::default();
        for mut group in due_together.into_values().filter(|group| group.len() > 1) {
            group.sort();
            for (order, (_, node)) in (1..).zip(group) {
                orders.insert(node, order);
            }
        }
        TickOrders(orders)
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
        let orders = TickOrders::new(self.graph);
        for node in self.graph.node_indices() {
            write!(f, "  ")?;
            write_node(f, self.graph, &names, &orders, node, true)?;
            writeln!(f)?;
        }
        write!(f, "}}")
    }
}

pub fn component(kind: NodeKind, state: NodeState) -> impl fmt::Display {
    Component { kind, state }
}

struct Component {
    kind: NodeKind,
    state: NodeState,
}

impl fmt::Display for Component {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (mut kind, mut state) = (self.kind, self.state);
        let mut fields = FieldPrinter::default();
        visit(&mut kind, &mut state, &mut fields);
        fields.write(f, kind_name(kind))
    }
}

pub fn write_node(
    f: &mut impl fmt::Write,
    graph: &Graph,
    names: &NodeNames,
    orders: &TickOrders,
    idx: NodeIdx,
    with_blocks: bool,
) -> fmt::Result {
    let node = &graph[idx];
    match node.names.as_slice() {
        [] => write!(f, "%{}", names.of(idx))?,
        all => {
            for (index, name) in all.iter().enumerate() {
                if index > 0 {
                    write!(f, ", ")?;
                }
                write!(f, "%{name}")?;
            }
        }
    }
    write!(f, " = {}", component(node.kind, node.state))?;
    if node.port != node.kind.traits().role {
        let (marker, _) = PORTS
            .iter()
            .find(|&&(_, port)| port == node.port)
            .expect("every port is listed");
        write!(f, " {marker}")?;
    }
    if let Some(queued) = node.pending_tick {
        let mut tick = queued.tick;
        let mut order = orders.0.get(&idx).copied();
        let mut fields = FieldPrinter::default();
        visit_tick(&mut tick, &mut order, &mut fields);
        write!(f, " ")?;
        fields.write(f, "tick")?;
    }
    if with_blocks && !node.blocks.is_empty() {
        write!(f, " at[")?;
        for (index, &(pos, id)) in node.blocks.iter().enumerate() {
            if index > 0 {
                write!(f, ", ")?;
            }
            let block = Block::from_id(id).expect("graph nodes hold known block states");
            write!(f, "({},{},{}) {block}", pos.x, pos.y, pos.z)?;
        }
        write!(f, "]")?;
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

#[derive(Default)]
struct FieldPrinter(Vec<String>);

impl FieldPrinter {
    fn write(&self, f: &mut impl fmt::Write, name: &str) -> fmt::Result {
        f.write_str(name)?;
        if !self.0.is_empty() {
            write!(f, "({})", self.0.join(", "))?;
        }
        Ok(())
    }
}

impl Fields for FieldPrinter {
    fn value<T: Value>(&mut self, name: &str, value: &mut T, default: Option<T>) {
        if default != Some(*value) {
            self.0.push(format!("{name}={value}"));
        }
    }

    fn optional<T: Value>(&mut self, name: &str, value: &mut Option<T>) {
        if let Some(value) = value {
            self.0.push(format!("{name}={value}"));
        }
    }

    fn flag(&mut self, name: &str, value: &mut bool) {
        if *value {
            self.0.push(name.to_owned());
        }
    }

    fn powered(&mut self, power: &mut SignalStrength) {
        match *power {
            SignalStrength::Zero => {}
            SignalStrength::Fifteen => self.0.push("powered".to_owned()),
            power => self.0.push(format!("power={power}")),
        }
    }
}
