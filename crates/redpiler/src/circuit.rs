use crate::engine::{Drive, DriveError, Input, Snapshot};
use crate::graph::{Direction, DriveKind, Graph, LinkKind, Node, NodeIdx, NodeKind, QueuedTick};
use crate::ports::Role;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::BlockPos;
use rustc_hash::FxHashMap;
use serde::{Deserialize, Serialize};
use std::{fmt, mem};

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

#[derive(Clone)]
pub struct Circuit {
    nodes: Box<[Node]>,
    inputs: LinkTable,
    outputs: LinkTable,
    nodes_by_position: FxHashMap<BlockPos, NodeId>,
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
        Ok(Circuit::from_checked_parts(
            graph.into_node_weights().collect(),
            &links,
        ))
    }

    /// An error names the node at fault by its index in `graph`.
    pub fn check(graph: &Graph) -> Result<(), InvalidCircuit> {
        check_node_count(graph.node_count())?;
        for index in graph.node_indices() {
            let node = &graph[index];
            check_node(node).map_err(|problem| InvalidCircuit::of(index.index(), problem))?;
            for edge in graph.edges(index, Direction::Outgoing) {
                let target = edge.target();
                check_link(
                    (index.index(), node),
                    (target.index(), &graph[target]),
                    edge.weight().weight,
                )?;
            }
        }
        Ok(())
    }

    fn from_checked_parts(nodes: Box<[Node]>, links: &[Link]) -> Circuit {
        let outputs = LinkTable::new(links, nodes.len(), |link| link.source);
        let inputs = LinkTable::new(&outputs.links, nodes.len(), |link| link.target);
        let mut nodes_by_position = FxHashMap::default();
        for (index, node) in nodes.iter().enumerate() {
            for &(pos, _) in &node.blocks {
                nodes_by_position.insert(pos, NodeId::new(index));
            }
        }
        Circuit {
            nodes,
            inputs,
            outputs,
            nodes_by_position,
        }
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

    pub fn node_at(&self, pos: BlockPos) -> Option<NodeId> {
        self.nodes_by_position.get(&pos).copied()
    }

    pub fn ports(&self) -> impl Iterator<Item = (NodeId, Role)> + '_ {
        self.ids().filter_map(|id| Some((id, self.node(id).port?)))
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

    pub fn check_input(&self, id: NodeId, input: Input) -> Result<Drive, DriveError> {
        let node = self.node(id);
        if !node.is_input() {
            return Err(DriveError::NotAnInput(id));
        }
        let accepted = match (node.kind.drive_kind(), input) {
            (Some(DriveKind::OnOff), Input::Set(strength)) => {
                matches!(strength, SignalStrength::Zero | SignalStrength::Fifteen)
            }
            (Some(DriveKind::OnOff), Input::Toggle) => true,
            (Some(DriveKind::Strength), Input::Set(_)) => true,
            (Some(DriveKind::Press), Input::Press) => true,
            _ => false,
        };
        if !accepted {
            return Err(DriveError::Unsupported { node: id, input });
        }
        Ok(Drive::new(id, input))
    }

    pub fn check_snapshot(&self, snapshot: &Snapshot) -> Result<(), SnapshotError> {
        if snapshot.states.len() != self.len() {
            return Err(SnapshotError::NodeCount {
                expected: self.len(),
                found: snapshot.states.len(),
            });
        }
        let mut ticked = vec![false; self.len()];
        for &(node, tick) in &snapshot.pending_ticks {
            if node.index() >= self.len() {
                return Err(SnapshotError::MissingNode(node));
            }
            if tick.delay == 0 {
                return Err(SnapshotError::ZeroDelay(node));
            }
            if mem::replace(&mut ticked[node.index()], true) {
                return Err(SnapshotError::DuplicateTick(node));
            }
            if !self.node(node).can_tick() {
                return Err(SnapshotError::Untickable(node));
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SnapshotError {
    NodeCount { expected: usize, found: usize },
    MissingNode(NodeId),
    ZeroDelay(NodeId),
    DuplicateTick(NodeId),
    Untickable(NodeId),
}

impl fmt::Display for SnapshotError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SnapshotError::NodeCount { expected, found } => {
                write!(f, "snapshot has {found} node states (expected {expected})")
            }
            SnapshotError::MissingNode(node) => {
                write!(f, "pending tick on missing node {}", node.index())
            }
            SnapshotError::ZeroDelay(node) => {
                write!(f, "pending tick of node {} has no delay", node.index())
            }
            SnapshotError::DuplicateTick(node) => {
                write!(f, "node {} has more than one pending tick", node.index())
            }
            SnapshotError::Untickable(node) => {
                write!(f, "node {} has a pending tick but can't tick", node.index())
            }
        }
    }
}

impl std::error::Error for SnapshotError {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InvalidCircuit {
    node: Option<usize>,
    problem: String,
}

impl InvalidCircuit {
    fn of(node: usize, problem: String) -> InvalidCircuit {
        InvalidCircuit {
            node: Some(node),
            problem,
        }
    }

    fn anywhere(problem: String) -> InvalidCircuit {
        InvalidCircuit {
            node: None,
            problem,
        }
    }

    pub fn node(&self) -> Option<usize> {
        self.node
    }

    pub fn problem(&self) -> &str {
        &self.problem
    }
}

impl fmt::Display for InvalidCircuit {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.node {
            Some(node) => write!(f, "invalid circuit: node {node} {}", self.problem),
            None => write!(f, "invalid circuit: {}", self.problem),
        }
    }
}

impl std::error::Error for InvalidCircuit {}

fn check_node_count(count: usize) -> Result<(), InvalidCircuit> {
    if u32::try_from(count).is_err() {
        return Err(InvalidCircuit::anywhere(format!(
            "{count} nodes exceed u32"
        )));
    }
    Ok(())
}

fn check_node(node: &Node) -> Result<(), String> {
    for &(pos, id) in &node.blocks {
        if Block::from_id(id).get_id() != id {
            return Err(format!("has an invalid block state {id} at {pos}"));
        }
    }
    match node.kind {
        NodeKind::Repeater { delay, .. } if !(1..=4).contains(&delay) => {
            return Err(format!("has a repeater delay of {delay}, not 1 to 4"));
        }
        NodeKind::NoteBlock { note, .. } if note > 24 => {
            return Err(format!("has note {note}, not 0 to 24"));
        }
        _ => {}
    }
    if node.is_input() && node.kind.drive_kind().is_none() {
        return Err("is an input but can't be driven".to_owned());
    }
    if let Some(QueuedTick { tick, .. }) = node.pending_tick {
        if tick.delay == 0 {
            return Err("has a pending tick without a delay".to_owned());
        }
        if !node.can_tick() {
            return Err("has a pending tick but can't tick".to_owned());
        }
    }
    Ok(())
}

fn check_link(
    (source_index, source): (usize, &Node),
    (target_index, target): (usize, &Node),
    weight: u8,
) -> Result<(), InvalidCircuit> {
    if weight >= 15 {
        return Err(InvalidCircuit::of(
            source_index,
            format!("has a link of weight {weight}, too long to carry power"),
        ));
    }
    if !source.kind.is_link_source() {
        return Err(InvalidCircuit::of(
            source_index,
            "can't have outgoing links".to_owned(),
        ));
    }
    if target.is_input() {
        return Err(InvalidCircuit::of(
            target_index,
            "is an input, so it can't have incoming links".to_owned(),
        ));
    }
    if !target.kind.is_link_target() {
        return Err(InvalidCircuit::of(
            target_index,
            "can't have incoming links".to_owned(),
        ));
    }
    Ok(())
}

#[derive(Clone)]
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
