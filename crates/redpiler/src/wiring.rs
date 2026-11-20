use crate::circuit::{Circuit, NodeId};
use crate::graph::{Direction, Graph, LinkKind, Node, NodeIdx};

#[derive(Debug, Clone, Copy)]
pub struct IncomingLink<Id> {
    pub source: Id,
    pub kind: LinkKind,
    pub weight: u8,
}

/// The nodes and links that a `Graph` and a `Circuit` both hold.
pub trait Wiring {
    type Id: Copy;

    fn ids(&self) -> impl Iterator<Item = Self::Id> + '_;

    /// Exceeds the index of every id.
    fn index_bound(&self) -> usize;

    fn index(id: Self::Id) -> usize;

    fn node(&self, id: Self::Id) -> &Node;

    fn links_into(&self, id: Self::Id) -> impl Iterator<Item = IncomingLink<Self::Id>> + '_;

    fn targets_of(&self, id: Self::Id) -> impl Iterator<Item = Self::Id> + '_;
}

impl Wiring for Graph {
    type Id = NodeIdx;

    fn ids(&self) -> impl Iterator<Item = NodeIdx> + '_ {
        self.node_indices()
    }

    fn index_bound(&self) -> usize {
        self.node_bound()
    }

    fn index(id: NodeIdx) -> usize {
        id.index()
    }

    fn node(&self, id: NodeIdx) -> &Node {
        &self[id]
    }

    fn links_into(&self, id: NodeIdx) -> impl Iterator<Item = IncomingLink<NodeIdx>> + '_ {
        let edges = self.edges(id, Direction::Incoming);
        edges.map(|edge| IncomingLink {
            source: edge.source(),
            kind: edge.weight().kind,
            weight: edge.weight().weight,
        })
    }

    fn targets_of(&self, id: NodeIdx) -> impl Iterator<Item = NodeIdx> + '_ {
        self.neighbors(id, Direction::Outgoing)
    }
}

impl Wiring for Circuit {
    type Id = NodeId;

    fn ids(&self) -> impl Iterator<Item = NodeId> + '_ {
        Circuit::ids(self)
    }

    fn index_bound(&self) -> usize {
        self.len()
    }

    fn index(id: NodeId) -> usize {
        id.index()
    }

    fn node(&self, id: NodeId) -> &Node {
        Circuit::node(self, id)
    }

    fn links_into(&self, id: NodeId) -> impl Iterator<Item = IncomingLink<NodeId>> + '_ {
        let links = self.inputs(id).iter();
        links.map(|link| IncomingLink {
            source: link.source,
            kind: link.kind,
            weight: link.weight,
        })
    }

    fn targets_of(&self, id: NodeId) -> impl Iterator<Item = NodeId> + '_ {
        self.outputs(id).iter().map(|link| link.target)
    }
}
