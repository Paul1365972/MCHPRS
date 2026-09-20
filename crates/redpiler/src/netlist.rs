use crate::compile_graph::{CompileGraph, CompileNode, LinkType};

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct NodeId(u32);

impl NodeId {
    pub fn new(index: usize) -> NodeId {
        NodeId(index.try_into().expect("node index exceeds u32"))
    }

    pub fn index(self) -> usize {
        self.0 as usize
    }

    pub(crate) const fn from_raw(raw: u32) -> NodeId {
        NodeId(raw)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Link {
    pub source: NodeId,
    pub target: NodeId,
    pub ty: LinkType,
    pub ss: u8,
}

pub struct Netlist {
    nodes: Box<[CompileNode]>,
    outputs: LinkTable,
    inputs: LinkTable,
}

impl Netlist {
    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    pub fn ids(&self) -> impl ExactSizeIterator<Item = NodeId> {
        (0..self.nodes.len()).map(NodeId::new)
    }

    pub fn node(&self, id: NodeId) -> &CompileNode {
        &self.nodes[id.index()]
    }

    pub fn outputs(&self, id: NodeId) -> &[Link] {
        self.outputs.of(id)
    }

    pub fn inputs(&self, id: NodeId) -> &[Link] {
        self.inputs.of(id)
    }
}

impl From<CompileGraph> for Netlist {
    fn from(graph: CompileGraph) -> Netlist {
        let (graph_nodes, graph_edges) = graph.into_parts();
        let bound = graph_nodes
            .iter()
            .map(|(index, _)| index.index() + 1)
            .max()
            .unwrap_or(0);
        let mut ids = vec![None; bound];
        let mut nodes = Vec::with_capacity(graph_nodes.len());
        for (index, node) in graph_nodes {
            ids[index.index()] = Some(NodeId::new(nodes.len()));
            nodes.push(node);
        }
        let links: Vec<Link> = graph_edges
            .into_iter()
            .map(|(source, target, link)| Link {
                source: ids[source.index()].expect("link endpoint exists"),
                target: ids[target.index()].expect("link endpoint exists"),
                ty: link.ty,
                ss: link.ss,
            })
            .collect();
        let outputs = LinkTable::new(
            links.clone(),
            nodes.len(),
            |link| link.source,
            |link| link.target,
        );
        let inputs = LinkTable::new(links, nodes.len(), |link| link.target, |link| link.source);
        Netlist {
            nodes: nodes.into_boxed_slice(),
            outputs,
            inputs,
        }
    }
}

struct LinkTable {
    links: Box<[Link]>,
    starts: Box<[usize]>,
}

impl LinkTable {
    fn new(
        mut links: Vec<Link>,
        node_count: usize,
        owner: fn(&Link) -> NodeId,
        peer: fn(&Link) -> NodeId,
    ) -> LinkTable {
        links.sort_by_key(|link| (owner(link).index(), peer(link).index()));
        let mut starts = vec![0; node_count + 1];
        for link in &links {
            starts[owner(link).index() + 1] += 1;
        }
        for node in 0..node_count {
            starts[node + 1] += starts[node];
        }
        LinkTable {
            links: links.into_boxed_slice(),
            starts: starts.into_boxed_slice(),
        }
    }

    fn of(&self, id: NodeId) -> &[Link] {
        &self.links[self.starts[id.index()]..self.starts[id.index() + 1]]
    }
}
