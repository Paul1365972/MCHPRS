use super::argument::ClientArgument;
use super::node::CommandGraph;
use super::registry::CommandRegistry;
use mchprs_commands::graph::{NodeId, NodeKind};
use mchprs_network::packets::clientbound::{
    CCommands, CCommandsNode, CDeclareCommandsNodeParser as Parser, ClientBoundPacket,
};
use mchprs_network::packets::PacketEncoder;
use rustc_hash::FxHashMap;

const LITERAL: i8 = 1;
const ARGUMENT: i8 = 2;
const EXECUTABLE: i8 = 4;
const REDIRECT: i8 = 8;
const SUGGESTIONS: i8 = 16;

#[derive(PartialEq)]
enum Declared {
    Root,
    Literal(String),
    Argument(Parser),
}

struct Projection<'a> {
    graph: &'a CommandGraph,
    nodes: Vec<CCommandsNode>,
    ids: FxHashMap<Vec<NodeId>, i32>,
}

impl Projection<'_> {
    fn declared(&self, id: NodeId) -> Declared {
        match &self.graph.node(id).kind {
            NodeKind::Root => Declared::Root,
            NodeKind::Literal(name) => Declared::Literal(name.clone()),
            NodeKind::Argument { parser, .. } => {
                Declared::Argument(match parser.client_argument() {
                    ClientArgument::Client(parser) | ClientArgument::AskServer(parser) => parser,
                    ClientArgument::Opaque => Parser::String(2),
                })
            }
        }
    }

    /// The client keeps the first of two siblings that fit, and siblings it reads the same way
    /// always both fit, so each such group reaches it as one node that accepts what any accepts
    fn groups(&self, children: impl IntoIterator<Item = NodeId>) -> Vec<Vec<NodeId>> {
        let mut groups: Vec<(Declared, Vec<NodeId>)> = Vec::new();
        for child in children {
            let declared = self.declared(child);
            match groups.iter_mut().find(|(other, _)| *other == declared) {
                Some((_, group)) => group.push(child),
                None => groups.push((declared, vec![child])),
            }
        }
        groups.into_iter().map(|(_, group)| group).collect()
    }

    fn visit(&mut self, group: Vec<NodeId>) -> i32 {
        if let Some(&index) = self.ids.get(&group) {
            return index;
        }
        let graph = self.graph;
        let index = self.nodes.len() as i32;
        self.ids.insert(group.clone(), index);
        self.nodes.push(CCommandsNode {
            flags: 0,
            children: Vec::new(),
            redirect_node: None,
            name: None,
            parser: None,
            suggestions_type: None,
        });
        let members: Vec<_> = group.iter().map(|&id| graph.node(id)).collect();
        let mut flags = if members.iter().any(|node| node.executor.is_some()) {
            EXECUTABLE
        } else {
            0
        };
        let mut tail = false;
        let (name, parser, suggestions_type) = match self.declared(group[0]) {
            Declared::Root => (None, None, None),
            Declared::Literal(name) => {
                flags |= LITERAL;
                (Some(name), None, None)
            }
            Declared::Argument(protocol) => {
                flags |= ARGUMENT;
                let clients: Vec<ClientArgument> = members
                    .iter()
                    .filter_map(|node| match &node.kind {
                        NodeKind::Argument { parser, .. } => Some(parser.client_argument()),
                        _ => None,
                    })
                    .collect();
                tail = clients
                    .iter()
                    .any(|client| matches!(client, ClientArgument::Opaque));
                if tail {
                    flags |= EXECUTABLE;
                }
                let suggests = clients
                    .iter()
                    .any(|client| !matches!(client, ClientArgument::Client(_)));
                if suggests {
                    flags |= SUGGESTIONS;
                }
                let mut names: Vec<&str> = Vec::new();
                for node in &members {
                    if !names.contains(&node.name()) {
                        names.push(node.name());
                    }
                }
                (
                    Some(names.join("|")),
                    Some(protocol),
                    suggests.then(|| "minecraft:ask_server".to_owned()),
                )
            }
        };
        let children: Vec<i32> = if tail {
            Vec::new()
        } else {
            let children = members
                .iter()
                .flat_map(|node| node.children().iter().copied());
            self.groups(children)
                .into_iter()
                .map(|group| self.visit(group))
                .collect()
        };
        // The vanilla client cancels its pending suggestion request when another sibling asks the
        // server.
        let mut has_suggestions = false;
        for &child in &children {
            let child = &mut self.nodes[child as usize];
            if child.suggestions_type.is_some() {
                if has_suggestions {
                    child.flags &= !SUGGESTIONS;
                    child.suggestions_type = None;
                }
                has_suggestions = true;
            }
        }
        let redirect_node = members[0].redirect().map(|target| self.visit(vec![target]));
        if redirect_node.is_some() {
            flags |= REDIRECT;
        }
        self.nodes[index as usize] = CCommandsNode {
            flags,
            children,
            redirect_node,
            name,
            parser,
            suggestions_type,
        };
        index
    }
}

impl CommandRegistry {
    fn declare_commands(&self) -> CCommands {
        let mut projection = Projection {
            graph: &self.graph,
            nodes: Vec::new(),
            ids: FxHashMap::default(),
        };
        let root = projection.visit(vec![self.graph.root()]);
        let mut children = Vec::new();
        for (name, node) in self.visible_commands() {
            if let Some(id) = node {
                children.push(projection.ids[&vec![id]]);
                continue;
            }
            let tail = projection.nodes.len() as i32;
            projection.nodes.push(CCommandsNode {
                flags: ARGUMENT | EXECUTABLE | SUGGESTIONS,
                children: Vec::new(),
                redirect_node: None,
                name: Some("arguments".to_owned()),
                parser: Some(Parser::String(2)),
                suggestions_type: Some("minecraft:ask_server".to_owned()),
            });
            let index = projection.nodes.len() as i32;
            let executable = self.dispatch(name).is_ok();
            projection.nodes.push(CCommandsNode {
                flags: LITERAL | if executable { EXECUTABLE } else { 0 },
                children: vec![tail],
                redirect_node: None,
                name: Some(name.to_owned()),
                parser: None,
                suggestions_type: None,
            });
            children.push(index);
        }
        projection.nodes[root as usize].children = children;
        CCommands {
            nodes: projection.nodes,
            root_index: root,
        }
    }

    pub fn declare_commands_packet(&self) -> PacketEncoder {
        self.declare_commands().encode()
    }
}
