use crate::engine::{DriveError, Input, Snapshot};
use crate::graph::{Direction, Drive, Graph, LinkKind, Node, NodeIdx, QueuedTick};
use crate::ports::Role;
use bincode::Options;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::{BlockPos, MC_DATA_VERSION};
use rustc_hash::FxHashMap;
use serde::{Deserialize, Serialize};
use std::{fmt, mem};

const MAGIC: [u8; 8] = *b"redpiler";
const FORMAT_VERSION: u32 = 1;

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

pub struct Circuit {
    nodes: Box<[Node]>,
    inputs: LinkTable,
    outputs: LinkTable,
    nodes_by_position: FxHashMap<BlockPos, NodeId>,
}

impl Circuit {
    /// Keeps the order of each node's outgoing links, which decides the order of updates.
    pub fn new(graph: Graph) -> Result<Circuit, InvalidCircuit> {
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
        Circuit::from_parts(graph.into_node_weights().collect(), &links)
    }

    fn from_parts(nodes: Box<[Node]>, links: &[Link]) -> Result<Circuit, InvalidCircuit> {
        check(&nodes, links).map_err(InvalidCircuit)?;
        let outputs = LinkTable::new(links, nodes.len(), |link| link.source);
        let inputs = LinkTable::new(&outputs.links, nodes.len(), |link| link.target);
        let mut nodes_by_position = FxHashMap::default();
        for (index, node) in nodes.iter().enumerate() {
            for &(position, _) in &node.blocks {
                nodes_by_position.insert(position, NodeId::new(index));
            }
        }
        Ok(Circuit {
            nodes,
            inputs,
            outputs,
            nodes_by_position,
        })
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

    pub fn node_at(&self, position: BlockPos) -> Option<NodeId> {
        self.nodes_by_position.get(&position).copied()
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

    pub fn check_input(&self, id: NodeId, input: Input) -> Result<(), DriveError> {
        let node = self.node(id);
        if !node.is_input() {
            return Err(DriveError::NotAnInput(id));
        }
        let accepted = match (node.kind.drive(), input) {
            (Some(Drive::OnOff), Input::Set(strength)) => {
                matches!(strength, SignalStrength::Zero | SignalStrength::Fifteen)
            }
            (Some(Drive::OnOff), Input::Toggle) => true,
            (Some(Drive::Strength), Input::Set(_)) => true,
            (Some(Drive::Press), Input::Press) => true,
            _ => false,
        };
        match accepted {
            true => Ok(()),
            false => Err(DriveError::Unsupported { node: id, input }),
        }
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
pub struct InvalidCircuit(String);

impl fmt::Display for InvalidCircuit {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "invalid circuit: {}", self.0)
    }
}

impl std::error::Error for InvalidCircuit {}

#[derive(Serialize)]
struct CircuitFileRef<'a> {
    nodes: &'a [Node],
    links: &'a [Link],
}

#[derive(Deserialize)]
struct CircuitFile {
    nodes: Vec<Node>,
    links: Vec<Link>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CircuitFormatError {
    NotACircuit,
    UnsupportedVersion(u32),
    UnsupportedDataVersion(i32),
    Malformed(String),
}

impl fmt::Display for CircuitFormatError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CircuitFormatError::NotACircuit => write!(f, "not a circuit"),
            CircuitFormatError::UnsupportedVersion(version) => write!(
                f,
                "unsupported circuit format version {version} (expected {FORMAT_VERSION})"
            ),
            CircuitFormatError::UnsupportedDataVersion(version) => write!(
                f,
                "unsupported Minecraft data version {version} (expected {MC_DATA_VERSION})"
            ),
            CircuitFormatError::Malformed(message) => write!(f, "malformed circuit: {message}"),
        }
    }
}

impl std::error::Error for CircuitFormatError {}

impl Circuit {
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = MAGIC.to_vec();
        bytes.extend(FORMAT_VERSION.to_le_bytes());
        bytes.extend(MC_DATA_VERSION.to_le_bytes());
        let file = CircuitFileRef {
            nodes: &self.nodes,
            links: &self.outputs.links,
        };
        bincode::options()
            .serialize_into(&mut bytes, &file)
            .expect("a circuit serializes");
        bytes
    }

    pub fn from_bytes(bytes: &[u8]) -> Result<Circuit, CircuitFormatError> {
        let rest = bytes
            .strip_prefix(&MAGIC)
            .ok_or(CircuitFormatError::NotACircuit)?;
        let (version, rest) = rest
            .split_first_chunk::<4>()
            .ok_or(CircuitFormatError::NotACircuit)?;
        let version = u32::from_le_bytes(*version);
        if version != FORMAT_VERSION {
            return Err(CircuitFormatError::UnsupportedVersion(version));
        }
        let (data_version, body) = rest
            .split_first_chunk::<4>()
            .ok_or(CircuitFormatError::NotACircuit)?;
        let data_version = i32::from_le_bytes(*data_version);
        if data_version != MC_DATA_VERSION {
            return Err(CircuitFormatError::UnsupportedDataVersion(data_version));
        }
        let file: CircuitFile = bincode::options()
            .deserialize(body)
            .map_err(|error| CircuitFormatError::Malformed(error.to_string()))?;
        Circuit::from_parts(file.nodes.into_boxed_slice(), &file.links)
            .map_err(|InvalidCircuit(message)| CircuitFormatError::Malformed(message))
    }
}

fn check(nodes: &[Node], links: &[Link]) -> Result<(), String> {
    let node_count = nodes.len();
    if u32::try_from(node_count).is_err() {
        return Err(format!("{node_count} nodes exceed u32"));
    }
    for link in links {
        if link.source.index() >= node_count || link.target.index() >= node_count {
            return Err(format!("a link joins a missing node: {link:?}"));
        }
        let target = &nodes[link.target.index()];
        if target.is_input() {
            return Err(format!(
                "input {} has an incoming link",
                link.target.index()
            ));
        }
        if !target.kind.is_link_target() {
            return Err(format!(
                "node {} ({:?}) can't be a link target",
                link.target.index(),
                target.kind
            ));
        }
        let source = &nodes[link.source.index()];
        if !source.kind.is_link_source() {
            return Err(format!(
                "node {} ({:?}) can't be a link source",
                link.source.index(),
                source.kind
            ));
        }
    }
    for (index, node) in nodes.iter().enumerate() {
        for &(position, id) in &node.blocks {
            if Block::from_id(id).get_id() != id {
                return Err(format!("invalid block state {id} at {position}"));
            }
        }
        if node.is_input() && node.kind.drive().is_none() {
            return Err(format!("input {index} can't be driven"));
        }
        if let Some(QueuedTick { tick, .. }) = node.pending_tick {
            if tick.delay == 0 {
                return Err(format!("pending tick of node {index} has no delay"));
            }
            if !node.can_tick() {
                return Err(format!(
                    "node {index} ({:?}) has a pending tick but can't tick",
                    node.kind
                ));
            }
        }
    }
    Ok(())
}

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
