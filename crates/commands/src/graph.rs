use crate::reader::{Reader, SyntaxError};

pub trait Argument: Clone + PartialEq {
    type Value: Clone;

    fn consumes_remaining(&self) -> bool {
        false
    }

    fn parse(&self, reader: &mut Reader<'_>) -> Result<Self::Value, SyntaxError>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct NodeId(usize);

#[derive(Debug, Clone, PartialEq)]
pub enum NodeKind<A> {
    Root,
    Literal(String),
    Argument { name: String, parser: A },
}

impl<A> NodeKind<A> {
    pub fn name(&self) -> &str {
        match self {
            Self::Root => "",
            Self::Literal(name) | Self::Argument { name, .. } => name,
        }
    }
}

impl<A: Argument> NodeKind<A> {
    pub(crate) fn parse(&self, reader: &mut Reader<'_>) -> Result<Option<A::Value>, SyntaxError> {
        let start = reader.cursor();
        let value = match self {
            Self::Literal(name) => {
                if reader.take_while(|ch| ch != ' ') != name {
                    return Err(SyntaxError::new(start, format!("Expected {name}")));
                }
                None
            }
            Self::Argument { parser, .. } => Some(parser.parse(reader)?),
            Self::Root => unreachable!(),
        };
        if reader.peek().is_some_and(|ch| ch != ' ') {
            return Err(reader.error("Expected whitespace between arguments"));
        }
        Ok(value)
    }
}

pub struct Node<A, E> {
    pub kind: NodeKind<A>,
    pub executor: Option<E>,
    pub(crate) children: Vec<NodeId>,
    pub(crate) redirect: Option<NodeId>,
}

impl<A, E> Node<A, E> {
    pub fn new(kind: NodeKind<A>, executor: Option<E>) -> Self {
        Self {
            kind,
            executor,
            children: Vec::new(),
            redirect: None,
        }
    }

    pub fn name(&self) -> &str {
        self.kind.name()
    }

    pub fn children(&self) -> &[NodeId] {
        &self.children
    }

    pub fn redirect(&self) -> Option<NodeId> {
        self.redirect
    }
}

#[derive(Debug, thiserror::Error)]
pub enum RegistrationError {
    #[error("invalid command node name {0:?}")]
    InvalidName(String),
    #[error("duplicate argument `{0}`")]
    RepeatedArgument(String),
    #[error("conflicting definitions for `{0}`")]
    Conflict(String),
    #[error("a redirect cannot have children")]
    RedirectWithChildren,
    #[error("a greedy argument cannot have children or a redirect")]
    GreedyContinuation,
    #[error("the root cannot redirect")]
    RootRedirect,
}

pub struct Graph<A, E> {
    pub(crate) nodes: Vec<Node<A, E>>,
}

impl<A, E> Default for Graph<A, E> {
    fn default() -> Self {
        Self {
            nodes: vec![Node::new(NodeKind::Root, None)],
        }
    }
}

impl<A, E> Graph<A, E> {
    pub fn root(&self) -> NodeId {
        NodeId(0)
    }

    pub fn node(&self, id: NodeId) -> &Node<A, E> {
        &self.nodes[id.0]
    }

    pub fn set_executor(&mut self, id: NodeId, executor: E) {
        self.nodes[id.0].executor = Some(executor);
    }

    pub fn child(&self, parent: NodeId, name: &str) -> Option<NodeId> {
        self.node(parent)
            .children
            .iter()
            .copied()
            .find(|&id| self.node(id).name() == name)
    }

    pub(crate) fn matching_literal(&self, parent: NodeId, input: &str) -> Option<NodeId> {
        let token = input.split(' ').next().unwrap_or_default();
        self.node(parent)
            .children()
            .iter()
            .copied()
            .find(|&id| matches!(&self.node(id).kind, NodeKind::Literal(name) if name == token))
    }
}

impl<A: Argument, E> Graph<A, E> {
    pub fn insert(
        &mut self,
        parent: NodeId,
        node: Node<A, E>,
    ) -> Result<NodeId, RegistrationError> {
        let parent_node = &self.nodes[parent.0];
        if matches!(&parent_node.kind, NodeKind::Argument { parser, .. } if parser.consumes_remaining())
        {
            return Err(RegistrationError::GreedyContinuation);
        }
        if parent_node.redirect.is_some() {
            return Err(RegistrationError::RedirectWithChildren);
        }
        if matches!(node.kind, NodeKind::Root)
            || node.name().is_empty()
            || node.name().chars().any(char::is_whitespace)
        {
            return Err(RegistrationError::InvalidName(node.name().to_owned()));
        }

        if let Some(id) = self.child(parent, node.name()) {
            let existing = &mut self.nodes[id.0];
            if existing.kind != node.kind || existing.redirect.is_some() {
                return Err(RegistrationError::Conflict(node.name().to_owned()));
            }
            if node.executor.is_some() {
                existing.executor = node.executor;
            }
            return Ok(id);
        }

        let id = NodeId(self.nodes.len());
        self.nodes.push(node);
        self.nodes[parent.0].children.push(id);
        Ok(id)
    }

    pub fn redirect(&mut self, id: NodeId, target: NodeId) -> Result<(), RegistrationError> {
        if id == self.root() {
            return Err(RegistrationError::RootRedirect);
        }

        let node = &mut self.nodes[id.0];
        if node.redirect.is_some_and(|existing| existing != target) {
            return Err(RegistrationError::Conflict(node.name().to_owned()));
        }
        if matches!(&node.kind, NodeKind::Argument { parser, .. } if parser.consumes_remaining()) {
            return Err(RegistrationError::GreedyContinuation);
        }
        if !node.children.is_empty() {
            return Err(RegistrationError::RedirectWithChildren);
        }

        node.redirect = Some(target);
        Ok(())
    }
}
