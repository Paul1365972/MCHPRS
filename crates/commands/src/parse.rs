use crate::graph::{Argument, Graph, NodeId, NodeKind};
use crate::reader::{Reader, SyntaxError};
use std::cmp::Reverse;

#[derive(Debug, Clone)]
pub struct ParsedArgument<V> {
    pub name: String,
    pub value: V,
}

#[derive(Debug, Clone)]
pub struct Context<V, E> {
    pub nodes: Vec<NodeId>,
    pub arguments: Vec<ParsedArgument<V>>,
    pub executor: Option<E>,
    pub child: Option<Box<Context<V, E>>>,
}

impl<V, E> Context<V, E> {
    fn new() -> Self {
        Self {
            nodes: Vec::new(),
            arguments: Vec::new(),
            executor: None,
            child: None,
        }
    }

    fn terminal(&self) -> &Self {
        self.child.as_ref().map_or(self, |child| child.terminal())
    }

    pub fn into_terminal(mut self) -> Self {
        while let Some(child) = self.child {
            self = *child;
        }
        self
    }

    pub fn path(&self) -> Vec<NodeId> {
        let mut nodes = self.nodes.clone();
        if let Some(child) = &self.child {
            nodes.extend(child.path());
        }
        nodes
    }
}

#[derive(Debug)]
pub struct Parse<V, E> {
    pub context: Context<V, E>,
    pub cursor: usize,
    pub errors: Vec<(NodeId, SyntaxError)>,
}

impl<V, E> Parse<V, E> {
    fn sole_error(&self) -> Option<&SyntaxError> {
        let ((_, first), rest) = self.errors.split_first()?;
        rest.iter()
            .all(|(_, error)| error.cursor == first.cursor && error.message == first.message)
            .then_some(first)
    }

    pub fn error(&self, input: &str) -> Option<SyntaxError> {
        if self.cursor < input.len() {
            if let Some(error) = self.sole_error() {
                return Some(error.clone());
            }
            return Some(SyntaxError::new(
                self.cursor,
                if self.context.nodes.is_empty() {
                    "Unknown command"
                } else {
                    "Incorrect argument"
                },
            ));
        }

        if self.context.terminal().executor.is_none() {
            return Some(SyntaxError::new(self.cursor, "Incomplete command"));
        }
        None
    }
}

impl<A: Argument, E: Clone> Graph<A, E> {
    pub fn parse(&self, input: &str) -> Parse<A::Value, E> {
        self.parse_nodes(self.root(), Reader::new(input), Context::new(), 0)
    }

    fn parse_nodes(
        &self,
        parent: NodeId,
        reader: Reader<'_>,
        context: Context<A::Value, E>,
        depth: usize,
    ) -> Parse<A::Value, E> {
        let start = reader.cursor();
        if depth > 256 {
            return Parse {
                context,
                cursor: start,
                errors: vec![(parent, reader.error("Command nesting limit exceeded"))],
            };
        }

        let literal = self.matching_literal(parent, reader.remaining());
        let candidates: Vec<_> = match literal {
            Some(id) => vec![id],
            None => self
                .node(parent)
                .children()
                .iter()
                .copied()
                .filter(|&id| matches!(self.node(id).kind, NodeKind::Argument { .. }))
                .collect(),
        };

        let mut errors = Vec::new();
        let mut potentials = Vec::new();
        for id in candidates {
            let node = self.node(id);
            let mut reader = reader;
            let value = match node.kind.parse(&mut reader) {
                Ok(value) => value,
                Err(error) => {
                    errors.push((id, error));
                    continue;
                }
            };
            let mut next = context.clone();
            if let Some(value) = value {
                next.arguments.push(ParsedArgument {
                    name: node.name().to_owned(),
                    value,
                });
            }
            next.nodes.push(id);
            next.executor = node.executor.clone();

            let minimum = if node.redirect().is_some() { 1 } else { 2 };
            if reader.remaining().len() >= minimum {
                reader.read();
                if let Some(target) = node.redirect() {
                    let child = self.parse_nodes(target, reader, Context::new(), depth + 1);
                    next.child = Some(Box::new(child.context));
                    return Parse {
                        context: next,
                        cursor: child.cursor,
                        errors: child.errors,
                    };
                }
                potentials.push(self.parse_nodes(id, reader, next, depth + 1));
            } else {
                potentials.push(Parse {
                    context: next,
                    cursor: reader.cursor(),
                    errors: Vec::new(),
                });
            }
        }

        potentials
            .into_iter()
            .min_by_key(|parse| {
                (
                    parse.cursor < reader.input().len(),
                    Reverse(parse.cursor),
                    parse.errors.is_empty(),
                )
            })
            .unwrap_or(Parse {
                context,
                cursor: start,
                errors,
            })
    }
}
