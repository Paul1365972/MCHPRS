use crate::behavior::{AcceptedInput, InputStrengths, Outcome, Pending};
use crate::graph::{self, Direction, Graph, LinkKind, Node, NodeIdx, NodeKind, QueuedTick};
use crate::nodes::Snapshot;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redstone::attached::BUTTON_PRESS_TICKS;
use mchprs_redstone::input::{Input, InputError};
use mchprs_redstone::tick_queue::PendingTick;
use mchprs_simulation::backend::STALE;
use rustc_hash::FxHashMap;
use serde::{Deserialize, Serialize};
use std::ops::RangeInclusive;

pub const REPEATER_DELAYS: RangeInclusive<u8> = 1..=4;
pub const NOTES: RangeInclusive<u8> = 0..=24;
pub const LINK_WEIGHTS: RangeInclusive<u8> = 0..=14;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct NodeId(u32);

impl NodeId {
    pub fn new(index: usize) -> NodeId {
        NodeId(u32::try_from(index).expect("node index exceeds u32"))
    }

    pub fn index(self) -> usize {
        self.0 as usize
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Link {
    pub source: NodeId,
    pub target: NodeId,
    pub kind: LinkKind,
    pub weight: u8,
}

#[derive(Debug, Clone)]
pub struct Circuit {
    nodes: Box<[Node]>,
    inputs: LinkTable,
    outputs: LinkTable,
}

impl Circuit {
    /// Keeps the order of each node's outgoing links, which decides the order of updates.
    pub fn new(graph: Graph) -> Result<Circuit, InvalidCircuit> {
        Circuit::check(&graph)?;
        let mut ids = vec![None; graph.node_bound()];
        for (index, node) in graph.node_indices().enumerate() {
            ids[node.index()] = Some(NodeId::new(index));
        }
        let id = |node: NodeIdx| ids[node.index()].expect("links join live nodes");
        let mut links = Vec::with_capacity(graph.edge_count());
        for node in graph.node_indices() {
            for edge in graph.edges(node, Direction::Outgoing) {
                links.push(Link {
                    source: id(edge.source()),
                    target: id(edge.target()),
                    kind: edge.weight().kind,
                    weight: edge.weight().weight,
                });
            }
        }
        let nodes: Box<[Node]> = graph.into_node_weights().collect();
        let outputs = LinkTable::new(&links, nodes.len(), |link| link.source);
        let inputs = LinkTable::new(&outputs.links, nodes.len(), |link| link.target);
        Ok(Circuit {
            nodes,
            inputs,
            outputs,
        })
    }

    pub fn check(graph: &Graph) -> Result<(), InvalidCircuit> {
        check_node_count(graph.node_count())?;
        let mut schedulable_ticks = SchedulableTicks::default();
        for index in graph.node_indices() {
            let node = &graph[index];
            let inputs = inputs_in(graph, index);
            check_node(node, &mut schedulable_ticks).map_err(|problem| InvalidCircuit::Node {
                node: index,
                problem,
            })?;
            if is_stale(node, inputs) {
                return Err(InvalidCircuit::Stale { node: index });
            }
            for edge in graph.edges(index, Direction::Outgoing) {
                let target = edge.target();
                check_link((index, node), (target, &graph[target]), *edge.weight())?;
            }
        }
        Ok(())
    }

    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    pub fn ids(&self) -> impl ExactSizeIterator<Item = NodeId> + use<> {
        (0..self.nodes.len()).map(NodeId::new)
    }

    pub fn node(&self, id: NodeId) -> &Node {
        &self.nodes[id.index()]
    }

    pub fn inputs(&self, id: NodeId) -> &[Link] {
        self.inputs.of(id)
    }

    pub fn outputs(&self, id: NodeId) -> &[Link] {
        self.outputs.of(id)
    }

    pub fn initial(&self) -> Snapshot {
        let mut queued: Vec<_> = self
            .ids()
            .filter_map(|id| Some((id, self.node(id).pending_tick?)))
            .collect();
        queued.sort_by_key(|(_, queued)| queued.order);
        Snapshot {
            states: self.nodes.iter().map(|node| node.state).collect(),
            pending_ticks: queued
                .into_iter()
                .map(|(id, queued)| (id, queued.tick))
                .collect(),
        }
    }

    pub fn input_kinds(&self) -> InputKinds {
        let inputs = self
            .nodes
            .iter()
            .map(|node| node.is_input().then_some(node.kind));
        InputKinds(inputs.collect())
    }
}

/// The one place that decides what input a node takes.
#[derive(Debug, Clone)]
pub struct InputKinds(Box<[Option<NodeKind>]>);

impl InputKinds {
    pub fn accept(&self, node: NodeId, input: Input) -> Result<AcceptedInput, InputError> {
        let kind = self.0.get(node.index()).copied().flatten();
        let kind = kind.ok_or(InputError::NotAnInput)?;
        kind.accept(input).ok_or(InputError::Unsupported(input))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum InvalidCircuit {
    #[error("invalid circuit: node {} {STALE}", node.index())]
    Stale { node: NodeIdx },
    #[error("invalid circuit: node {} {problem}", node.index())]
    Node { node: NodeIdx, problem: String },
    #[error("invalid circuit: {0} nodes exceed u32")]
    TooManyNodes(usize),
}

pub fn inputs_in(graph: &Graph, node: NodeIdx) -> InputStrengths {
    let mut inputs = InputStrengths::default();
    for edge in graph.edges(node, Direction::Incoming) {
        let link = edge.weight();
        let carried = graph[edge.source()].state.power.saturating_sub(link.weight);
        inputs.raise(link.kind, carried);
    }
    inputs
}

pub fn is_stale(node: &Node, inputs: InputStrengths) -> bool {
    let pending = node.pending();
    let stays = Outcome::of(node.state);
    if node.is_input() {
        pending == Pending::None && node.can_tick() && node.kind.tick(node.state, inputs) != stays
    } else {
        node.kind.update(node.state, inputs, pending) != stays
    }
}

fn check_node_count(count: usize) -> Result<(), InvalidCircuit> {
    if u32::try_from(count).is_err() {
        return Err(InvalidCircuit::TooManyNodes(count));
    }
    Ok(())
}

#[derive(Default)]
struct SchedulableTicks {
    by_kind: FxHashMap<NodeKind, Vec<PendingTick>>,
}

impl SchedulableTicks {
    fn check(&mut self, kind: NodeKind, tick: PendingTick) -> Result<(), String> {
        let schedulable_ticks = self.by_kind.entry(kind);
        let longest_delay = schedulable_ticks
            .or_insert_with(|| kind.schedulable_ticks())
            .iter()
            .filter(|schedulable| schedulable.priority == tick.priority)
            .map(|schedulable| schedulable.delay)
            .max();
        match longest_delay {
            None => Err(format!(
                "has a pending tick of priority {}, which it never schedules",
                tick.priority
            )),
            Some(longest_delay) if tick.delay > longest_delay => Err(format!(
                "has a pending tick with a delay of {}, longer than the {longest_delay} it \
                 schedules with priority {}",
                tick.delay, tick.priority
            )),
            Some(_) => Ok(()),
        }
    }
}

fn check_node(node: &Node, schedulable_ticks: &mut SchedulableTicks) -> Result<(), String> {
    for &(pos, id) in &node.blocks {
        if Block::from_id(id).is_none() {
            return Err(format!("has an invalid block state {id} at {pos}"));
        }
    }
    match node.kind {
        NodeKind::Repeater { delay, .. } if !REPEATER_DELAYS.contains(&delay) => {
            let (min, max) = REPEATER_DELAYS.into_inner();
            return Err(format!(
                "has a repeater delay of {delay}, not {min} to {max}"
            ));
        }
        NodeKind::Button { press_ticks } if !BUTTON_PRESS_TICKS.contains(&press_ticks) => {
            let [stone, wooden] = BUTTON_PRESS_TICKS;
            return Err(format!(
                "presses for {press_ticks} ticks, not the {stone} or {wooden} of vanilla's buttons"
            ));
        }
        NodeKind::NoteBlock { note, .. } if !NOTES.contains(&note) => {
            let (min, max) = NOTES.into_inner();
            return Err(format!("has note {note}, not {min} to {max}"));
        }
        _ => {}
    }
    let traits = node.kind.traits();
    let power = node.state.power;
    if traits.is_on_off() && !matches!(power, SignalStrength::Zero | SignalStrength::Fifteen) {
        return Err(format!(
            "is on or off, so it can't have a power of {}",
            power.get()
        ));
    }
    if node.is_input() && traits.drive.is_none() {
        return Err("is an input but can't be driven".to_owned());
    }
    if node.state.repeater_locked && !traits.lockable {
        return Err("is locked but can't lock".to_owned());
    }
    if node.state.plate_touched && node.kind != NodeKind::PressurePlate {
        return Err("is touched but is no pressure plate".to_owned());
    }
    if let Some(QueuedTick { tick, .. }) = node.pending_tick {
        if tick.delay == 0 {
            return Err("has a pending tick without a delay".to_owned());
        }
        if !node.can_tick() {
            return Err("has a pending tick but can't tick".to_owned());
        }
        schedulable_ticks.check(node.kind, tick)?;
    }
    Ok(())
}

fn check_link(
    (source_index, source): (NodeIdx, &Node),
    (target_index, target): (NodeIdx, &Node),
    graph::Link { kind, weight }: graph::Link,
) -> Result<(), InvalidCircuit> {
    if !LINK_WEIGHTS.contains(&weight) {
        return Err(InvalidCircuit::Node {
            node: source_index,
            problem: format!("has a link of weight {weight}, too long to carry power"),
        });
    }
    if source.kind.traits().emits.is_none() {
        return Err(InvalidCircuit::Node {
            node: source_index,
            problem: "can't have outgoing links".to_owned(),
        });
    }
    if target.is_input() {
        return Err(InvalidCircuit::Node {
            node: target_index,
            problem: "is an input, so it can't have incoming links".to_owned(),
        });
    }
    if target.kind.traits().reads.is_none() {
        return Err(InvalidCircuit::Node {
            node: target_index,
            problem: "can't have incoming links".to_owned(),
        });
    }
    if kind == LinkKind::Side && !target.kind.traits().side_inputs {
        return Err(InvalidCircuit::Node {
            node: target_index,
            problem: "can't have incoming side links".to_owned(),
        });
    }
    Ok(())
}

#[derive(Debug, Clone)]
struct LinkTable {
    links: Vec<Link>,
    offsets: Vec<usize>,
}

impl LinkTable {
    fn new(links: &[Link], node_count: usize, owner: fn(&Link) -> NodeId) -> LinkTable {
        let mut offsets = vec![0; node_count + 1];
        for link in links {
            offsets[owner(link).index() + 1] += 1;
        }
        for index in 0..node_count {
            offsets[index + 1] += offsets[index];
        }
        let mut next = offsets.clone();
        let mut grouped = links.to_vec();
        for link in links {
            let slot = &mut next[owner(link).index()];
            grouped[*slot] = *link;
            *slot += 1;
        }
        LinkTable {
            links: grouped,
            offsets,
        }
    }

    fn of(&self, id: NodeId) -> &[Link] {
        &self.links[self.offsets[id.index()]..self.offsets[id.index() + 1]]
    }
}
