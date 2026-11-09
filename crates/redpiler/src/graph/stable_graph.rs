use std::{iter, slice};

#[derive(Copy, Clone, Hash, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct NodeIndex(u32);

impl NodeIndex {
    pub fn index(self) -> usize {
        self.0 as usize
    }

    pub fn new(idx: usize) -> Self {
        Self(u32::try_from(idx).expect("limit for index type reached"))
    }
}

#[derive(Copy, Clone, Hash, PartialEq, Eq, Debug)]
pub struct EdgeIndex(u32);

impl EdgeIndex {
    pub fn index(self) -> usize {
        self.0 as usize
    }

    pub fn new(idx: usize) -> Self {
        Self(u32::try_from(idx).expect("limit for index type reached"))
    }
}

#[derive(Clone)]
struct RawNode<Node> {
    value: Option<Node>,

    // [[first_incoming, last_incoming], [first_outgoing, last_outgoing]]
    edge_dirs: [Option<[EdgeIndex; 2]>; 2],
}

impl<Node> Default for RawNode<Node> {
    fn default() -> Self {
        Self {
            value: None,
            edge_dirs: Default::default(),
        }
    }
}

#[derive(Clone)]
struct EdgeDefinition<Edge> {
    value: Edge,
    endpoints: [NodeIndex; 2],
}

const DIR_INCOMING: usize = 0;
const DIR_OUTGOING: usize = 1;
const DIR_PREV: usize = 0;
const DIR_NEXT: usize = 1;

#[derive(Clone)]
struct RawEdge<Edge> {
    def: Option<EdgeDefinition<Edge>>,

    /// [[prev_incoming, next_incoming], [prev_outgoing, next_outgoing]]
    dirs: [[Option<EdgeIndex>; 2]; 2],
}

impl<Edge> Default for RawEdge<Edge> {
    fn default() -> Self {
        Self {
            def: None,
            dirs: Default::default(),
        }
    }
}

#[derive(Clone)]
pub struct StableGraph<Node, Edge> {
    nodes: Vec<RawNode<Node>>,
    edges: Vec<RawEdge<Edge>>,

    free_node_stack: Vec<NodeIndex>,
    free_edge_stack: Vec<EdgeIndex>,

    node_count: usize,
    edge_count: usize,
}

impl<Node, Edge> Default for StableGraph<Node, Edge> {
    fn default() -> Self {
        Self {
            nodes: Default::default(),
            edges: Default::default(),
            free_node_stack: Default::default(),
            free_edge_stack: Default::default(),
            node_count: Default::default(),
            edge_count: Default::default(),
        }
    }
}

impl<Node, Edge> StableGraph<Node, Edge> {
    pub fn node_count(&self) -> usize {
        self.node_count
    }

    pub fn edge_count(&self) -> usize {
        self.edge_count
    }

    pub fn add_node(&mut self, value: Node) -> NodeIndex {
        let idx = match self.free_node_stack.pop() {
            Some(idx) => idx,
            None => {
                let idx = NodeIndex::new(self.nodes.len());
                self.nodes.push(Default::default());
                idx
            }
        };
        self.nodes[idx.index()].value = Some(value);
        self.node_count += 1;
        idx
    }

    pub(super) fn add_edge(&mut self, from: NodeIndex, to: NodeIndex, value: Edge) -> EdgeIndex {
        if !self.contains_node(from) || !self.contains_node(to) {
            panic!("endpoint not in graph")
        }

        let idx = match self.free_edge_stack.pop() {
            Some(idx) => idx,
            None => {
                let idx = EdgeIndex::new(self.edges.len());
                self.edges.push(Default::default());
                idx
            }
        };
        self.edges[idx.index()].def = Some(EdgeDefinition {
            value,
            endpoints: [from, to],
        });

        let mut prevs = [None; 2];
        for (dir, node_idx) in [(DIR_INCOMING, to), (DIR_OUTGOING, from)] {
            let node = &mut self.nodes[node_idx.index()];
            if let Some([_, last]) = &mut node.edge_dirs[dir] {
                self.edges[last.index()].dirs[dir][DIR_NEXT] = Some(idx);
                prevs[dir] = Some(*last);
                *last = idx;
            } else {
                node.edge_dirs[dir] = Some([idx, idx]);
            }
        }
        self.edges[idx.index()].dirs[DIR_INCOMING][DIR_PREV] = prevs[DIR_INCOMING];
        self.edges[idx.index()].dirs[DIR_OUTGOING][DIR_PREV] = prevs[DIR_OUTGOING];

        self.edge_count += 1;
        idx
    }

    pub(super) fn edge_mut(&mut self, idx: EdgeIndex) -> &mut Edge {
        &mut self.edges[idx.index()].def.as_mut().unwrap().value
    }

    pub fn remove_edge(&mut self, idx: EdgeIndex) -> Option<Edge> {
        let edge = self.edges.get_mut(idx.index())?;
        let def = edge.def.take()?;
        let dirs = std::mem::take(&mut edge.dirs);

        for dir in [DIR_INCOMING, DIR_OUTGOING] {
            let endpoint_idx = def.endpoints[1 - dir];
            let endpoint = &mut self.nodes[endpoint_idx.index()];

            let prev = dirs[dir][DIR_PREV];
            let next = dirs[dir][DIR_NEXT];

            match (prev, next) {
                (None, None) => {
                    endpoint.edge_dirs[dir] = None;
                }
                (None, Some(next_edge)) => {
                    let [_, last] = endpoint.edge_dirs[dir].unwrap();
                    endpoint.edge_dirs[dir] = Some([next_edge, last]);
                }
                (Some(prev_edge), None) => {
                    let [first, _] = endpoint.edge_dirs[dir].unwrap();
                    endpoint.edge_dirs[dir] = Some([first, prev_edge]);
                }
                (Some(_), Some(_)) => {
                    // Neither the first nor last edge changes.
                }
            }
        }

        for dir in [DIR_INCOMING, DIR_OUTGOING] {
            let prev = dirs[dir][DIR_PREV];
            let next = dirs[dir][DIR_NEXT];

            if let Some(prev_edge) = prev {
                self.edges[prev_edge.index()].dirs[dir][DIR_NEXT] = next;
            }
            if let Some(next_edge) = next {
                self.edges[next_edge.index()].dirs[dir][DIR_PREV] = prev;
            }
        }

        self.free_edge_stack.push(idx);
        self.edge_count -= 1;
        Some(def.value)
    }

    /// Returns the edges in their order, each with the node at its other end.
    pub fn remove_edges(&mut self, node_idx: NodeIndex, dir: Direction) -> Vec<(NodeIndex, Edge)> {
        let mut removed = Vec::new();
        let mut edges = self.neighbors(node_idx, dir).detach();
        while let Some((edge_idx, other)) = edges.next(self) {
            let edge = self.remove_edge(edge_idx).expect("the edge exists");
            removed.push((other, edge));
        }
        removed
    }

    pub fn remove_node(&mut self, idx: NodeIndex) -> Option<Node> {
        let node = self.nodes.get_mut(idx.index())?;
        let value = node.value.take()?;

        for dir in [DIR_INCOMING, DIR_OUTGOING] {
            // TODO: make faster
            loop {
                let node = self.nodes.get_mut(idx.index())?;
                if let Some(range) = node.edge_dirs[dir] {
                    self.remove_edge(range[0]);
                } else {
                    break;
                }
            }
        }

        self.free_node_stack.push(idx);
        self.node_count -= 1;
        Some(value)
    }

    pub fn contains_node(&self, node_idx: NodeIndex) -> bool {
        self.nodes
            .get(node_idx.index())
            .is_some_and(|node| node.value.is_some())
    }

    pub fn node_bound(&self) -> usize {
        self.nodes.len()
    }

    pub fn node_indices(&self) -> NodeIndices<'_, Node> {
        NodeIndices {
            iter: self.nodes.iter().enumerate(),
        }
    }

    pub fn neighbors(&self, node_idx: NodeIndex, dir: Direction) -> Neighbors<'_, Edge> {
        let node = &self.nodes[node_idx.index()];
        if node.value.is_none() {
            panic!("node not present");
        }
        let first = node.edge_dirs[dir.dir()].map(|[first, _]| first);
        Neighbors {
            edges: &self.edges,
            dir: dir.dir(),
            next: first,
        }
    }

    pub fn edges(&self, node_idx: NodeIndex, dir: Direction) -> Edges<'_, Edge> {
        let node = &self.nodes[node_idx.index()];
        if node.value.is_none() {
            panic!("node not present");
        }
        let first = node.edge_dirs[dir.dir()].map(|[first, _]| first);
        Edges {
            edges: &self.edges,
            dir: dir.dir(),
            next: first,
        }
    }

    pub fn sort_edges_by_key<K: Ord>(
        &mut self,
        node_idx: NodeIndex,
        dir: Direction,
        mut key: impl FnMut(&Self, EdgeRef<'_, Edge>) -> K,
    ) {
        let mut edges: Vec<(K, EdgeIndex)> = self
            .edges(node_idx, dir)
            .map(|edge| {
                let idx = edge.id();
                (key(self, edge), idx)
            })
            .collect();
        edges.sort_by(|(a, _), (b, _)| a.cmp(b));
        let (Some((_, first)), Some((_, last))) = (edges.first(), edges.last()) else {
            return;
        };
        self.nodes[node_idx.index()].edge_dirs[dir.dir()] = Some([*first, *last]);
        for (i, (_, idx)) in edges.iter().enumerate() {
            let links = &mut self.edges[idx.index()].dirs[dir.dir()];
            links[DIR_PREV] = i.checked_sub(1).map(|prev| edges[prev].1);
            links[DIR_NEXT] = edges.get(i + 1).map(|(_, next)| *next);
        }
    }

    /// In the order of `node_indices`
    pub fn into_node_weights(self) -> impl Iterator<Item = Node> {
        self.nodes.into_iter().filter_map(|node| node.value)
    }

    pub fn retain_nodes<F>(&mut self, mut f: F)
    where
        F: FnMut(NodeIndex) -> bool,
    {
        for idx in 0..self.nodes.len() {
            if self.nodes[idx].value.is_none() {
                continue;
            }
            let idx = NodeIndex::new(idx);
            let keep = f(idx);
            if !keep {
                self.remove_node(idx);
            }
        }
    }
}

impl<Node, Edge> std::ops::Index<NodeIndex> for StableGraph<Node, Edge> {
    type Output = Node;

    fn index(&self, index: NodeIndex) -> &Self::Output {
        self.nodes[index.index()].value.as_ref().unwrap()
    }
}

impl<Node, Edge> std::ops::IndexMut<NodeIndex> for StableGraph<Node, Edge> {
    fn index_mut(&mut self, index: NodeIndex) -> &mut Self::Output {
        self.nodes[index.index()].value.as_mut().unwrap()
    }
}

impl<Node, Edge> std::ops::Index<EdgeIndex> for StableGraph<Node, Edge> {
    type Output = Edge;

    fn index(&self, index: EdgeIndex) -> &Self::Output {
        &self.edges[index.index()].def.as_ref().unwrap().value
    }
}

#[derive(Copy, Clone, Debug)]
pub enum Direction {
    Incoming,
    Outgoing,
}

impl Direction {
    fn dir(self) -> usize {
        match self {
            Direction::Incoming => DIR_INCOMING,
            Direction::Outgoing => DIR_OUTGOING,
        }
    }
}

pub struct NodeIndices<'a, Node: 'a> {
    iter: iter::Enumerate<slice::Iter<'a, RawNode<Node>>>,
}

impl<Node> Iterator for NodeIndices<'_, Node> {
    type Item = NodeIndex;

    fn next(&mut self) -> Option<Self::Item> {
        self.iter
            .find_map(|(idx, node)| node.value.as_ref().map(|_| NodeIndex::new(idx)))
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let (lower, _) = self.iter.size_hint();
        (lower, None)
    }
}

pub struct Neighbors<'a, Edge> {
    edges: &'a [RawEdge<Edge>],
    dir: usize,
    next: Option<EdgeIndex>,
}

impl<Edge> Iterator for Neighbors<'_, Edge> {
    type Item = NodeIndex;

    fn next(&mut self) -> Option<Self::Item> {
        let edge = &self.edges[self.next?.index()];
        let def = edge.def.as_ref().unwrap();

        self.next = edge.dirs[self.dir][DIR_NEXT];
        Some(def.endpoints[self.dir])
    }
}

impl<Edge> Neighbors<'_, Edge> {
    pub fn detach(&self) -> NeighborsDetached {
        NeighborsDetached {
            dir: self.dir,
            next: self.next,
        }
    }
}

pub struct NeighborsDetached {
    dir: usize,
    next: Option<EdgeIndex>,
}

impl NeighborsDetached {
    pub fn next<Node, Edge>(
        &mut self,
        graph: &StableGraph<Node, Edge>,
    ) -> Option<(EdgeIndex, NodeIndex)> {
        let edge_idx = self.next?;
        let edge = &graph.edges[edge_idx.index()];
        let def = edge.def.as_ref().unwrap();

        self.next = edge.dirs[self.dir][DIR_NEXT];
        Some((edge_idx, def.endpoints[self.dir]))
    }
}

pub struct EdgeRef<'a, Edge> {
    idx: EdgeIndex,
    edge: &'a EdgeDefinition<Edge>,
}

impl<Edge> EdgeRef<'_, Edge> {
    pub fn id(&self) -> EdgeIndex {
        self.idx
    }

    pub fn weight(&self) -> &'_ Edge {
        &self.edge.value
    }

    pub fn source(&self) -> NodeIndex {
        self.edge.endpoints[0]
    }

    pub fn target(&self) -> NodeIndex {
        self.edge.endpoints[1]
    }
}

impl<'a, Edge> Iterator for Edges<'a, Edge> {
    type Item = EdgeRef<'a, Edge>;

    fn next(&mut self) -> Option<Self::Item> {
        let idx = self.next?;
        let edge = &self.edges[idx.index()];
        let def = edge.def.as_ref().unwrap();

        self.next = edge.dirs[self.dir][DIR_NEXT];
        Some(EdgeRef { idx, edge: def })
    }
}

pub struct Edges<'a, Edge> {
    edges: &'a [RawEdge<Edge>],
    dir: usize,
    next: Option<EdgeIndex>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_graph() {
        let g: StableGraph<usize, usize> = StableGraph::default();
        assert_eq!(g.node_count(), 0);
        assert_eq!(g.edge_count(), 0);
    }

    #[test]
    fn test_add_nodes() {
        let mut g: StableGraph<usize, usize> = StableGraph::default();

        let n1 = g.add_node(10);
        let n2 = g.add_node(20);

        assert_ne!(n1, n2);
        assert_eq!(g.node_count(), 2);
    }

    #[test]
    fn test_add_edges() {
        let mut g: StableGraph<usize, usize> = StableGraph::default();

        let n1 = g.add_node(1);
        let n2 = g.add_node(2);

        let e = g.add_edge(n1, n2, 100);

        assert_eq!(g.edge_count(), 1);
        // sanity: edge index exists (not much else we can assert without getters)
        let removed = g.remove_edge(e);
        assert_eq!(removed, Some(100));
        assert_eq!(g.edge_count(), 0);
    }

    #[test]
    fn test_remove_node() {
        let mut g: StableGraph<usize, usize> = StableGraph::default();

        let n1 = g.add_node(42);
        let n2 = g.add_node(99);

        assert_eq!(g.node_count(), 2);

        let removed = g.remove_node(n1);
        assert_eq!(removed, Some(42));
        assert_eq!(g.node_count(), 1);

        let removed_again = g.remove_node(n1);
        assert_eq!(removed_again, None);

        // ensure other node still exists
        let removed_n2 = g.remove_node(n2);
        assert_eq!(removed_n2, Some(99));
        assert_eq!(g.node_count(), 0);
    }

    #[test]
    fn test_remove_edge() {
        let mut g: StableGraph<usize, usize> = StableGraph::default();

        let n1 = g.add_node(1);
        let n2 = g.add_node(2);

        let e1 = g.add_edge(n1, n2, 5);
        let e2 = g.add_edge(n2, n1, 10);

        assert_eq!(g.edge_count(), 2);

        assert_eq!(g.remove_edge(e1), Some(5));
        assert_eq!(g.edge_count(), 1);

        assert_eq!(g.remove_edge(e1), None); // already removed

        assert_eq!(g.remove_edge(e2), Some(10));
        assert_eq!(g.edge_count(), 0);
    }

    #[test]
    fn test_remove_node_removes_edges() {
        let mut g: StableGraph<usize, usize> = StableGraph::default();

        let n1 = g.add_node(1);
        let n2 = g.add_node(2);
        let n3 = g.add_node(3);

        let e1 = g.add_edge(n1, n2, 10);
        let e2 = g.add_edge(n2, n3, 20);
        let e3 = g.add_edge(n1, n3, 30);

        assert_eq!(g.edge_count(), 3);

        // removing n2 should remove edges involving it
        g.remove_node(n2);

        // e1 and e2 should be gone
        assert_eq!(g.remove_edge(e1), None);
        assert_eq!(g.remove_edge(e2), None);

        // e3 should still exist
        assert_eq!(g.remove_edge(e3), Some(30));
        assert_eq!(g.edge_count(), 0);
    }

    #[test]
    fn test_multiple_add_remove_cycles() {
        let mut g: StableGraph<usize, usize> = StableGraph::default();

        let n1 = g.add_node(1);
        let n2 = g.add_node(2);

        let e1 = g.add_edge(n1, n2, 100);
        assert_eq!(g.edge_count(), 1);

        g.remove_edge(e1);
        assert_eq!(g.edge_count(), 0);

        let e2 = g.add_edge(n2, n1, 200);
        assert_eq!(g.edge_count(), 1);

        assert_eq!(g.remove_edge(e2), Some(200));
        assert_eq!(g.edge_count(), 0);
    }
}
