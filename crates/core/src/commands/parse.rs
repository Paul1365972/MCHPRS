use super::argument::Parsed;
use super::executor::Executor;
use super::graph::{Graph, NodeId, NodeKind};
use mchprs_syntax::expect::Expect;
use mchprs_syntax::reader::{Reader, SyntaxError};
use mchprs_syntax::suggestion::Suggestion;
use std::cmp::Reverse;

#[derive(Clone)]
pub struct Context {
    pub nodes: Vec<NodeId>,
    pub arguments: Vec<Parsed>,
    pub executor: Option<Executor>,
}

impl Context {
    fn new() -> Self {
        Self {
            nodes: Vec::new(),
            arguments: Vec::new(),
            executor: None,
        }
    }
}

pub struct Parse {
    pub context: Context,
    pub cursor: usize,
    errors: Vec<SyntaxError>,
}

impl Parse {
    pub fn error(&self, input: &str) -> Option<SyntaxError> {
        if self.cursor < input.len() {
            return Some(self.stop_error());
        }
        if self.context.executor.is_none() {
            return Some(SyntaxError::new(self.cursor, "Incomplete command"));
        }
        None
    }

    fn stop_error(&self) -> SyntaxError {
        let precedence = |error: &SyntaxError| (error.cursor, error.recognized);
        let Some(foremost) = self
            .errors
            .iter()
            .filter(|error| error.cursor >= self.cursor)
            .max_by_key(|error| precedence(error))
        else {
            return self.mismatch(self.cursor);
        };
        let agreed = self
            .errors
            .iter()
            .filter(|error| precedence(error) == precedence(foremost))
            .all(|error| error.message == foremost.message);
        if agreed {
            foremost.clone()
        } else {
            self.mismatch(foremost.cursor)
        }
    }

    fn mismatch(&self, cursor: usize) -> SyntaxError {
        SyntaxError::new(
            cursor,
            if self.context.nodes.is_empty() {
                "Unknown command"
            } else {
                "Incorrect argument"
            },
        )
    }
}

#[derive(Debug)]
pub struct Offered {
    pub path: Vec<NodeId>,
    pub node: NodeId,
    pub start: usize,
    pub offers: Vec<Suggestion>,
}

impl Graph {
    pub fn parse(&self, input: &str) -> Parse {
        self.parse_nodes(self.root(), Reader::new(input), Context::new(), &mut None)
    }

    pub fn suggest(&self, input: &str) -> Vec<Offered> {
        let mut offered = Some(Vec::new());
        self.parse_nodes(
            self.root(),
            Reader::new(input),
            Context::new(),
            &mut offered,
        );
        offered.unwrap_or_default()
    }

    fn parse_nodes(
        &self,
        parent: NodeId,
        reader: Reader<'_>,
        context: Context,
        offered: &mut Option<Vec<Offered>>,
    ) -> Parse {
        let start = reader.cursor();
        if context.nodes.len() > 256 {
            return Parse {
                context,
                cursor: start,
                errors: vec![reader.error("Command nesting limit exceeded")],
            };
        }

        let children = self.node(parent).children();
        let candidates: Vec<_> = if offered.is_some() && !reader.remaining().contains(' ') {
            children.to_vec()
        } else if let Some(literal) = self.matching_literal(parent, reader.remaining()) {
            vec![literal]
        } else {
            children
                .iter()
                .copied()
                .filter(|&id| matches!(self.node(id).kind, NodeKind::Argument { .. }))
                .collect()
        };

        let mut errors = Vec::new();
        if children.is_empty() {
            errors.push(reader.error("Unexpected argument"));
        }
        let mut potentials = Vec::new();
        for id in candidates {
            let node = self.node(id);
            let mut reader = reader;
            let mut expect = if offered.is_some() {
                Expect::suggesting()
            } else {
                Expect::executing()
            };
            let parsed = node
                .kind
                .parse(&mut reader, &mut expect)
                .map_err(|error| expect.error(error));
            if let Some(sink) = offered.as_mut()
                && let Some((start, offers)) = expect.into_completion()
            {
                sink.push(Offered {
                    path: context.nodes.clone(),
                    node: id,
                    start,
                    offers,
                });
            }
            let value = match parsed {
                Ok(value) => value,
                Err(error) => {
                    errors.push(error);
                    continue;
                }
            };
            let mut next = context.clone();
            if let Some(value) = value {
                next.arguments.push(value);
            }
            next.nodes.push(id);
            next.executor = node.executor.clone();

            let minimum = if node.redirect().is_some() || offered.is_some() {
                1
            } else {
                2
            };
            if reader.remaining().len() < minimum {
                potentials.push(Parse {
                    context: next,
                    cursor: reader.cursor(),
                    errors: Vec::new(),
                });
                continue;
            }
            reader.read();
            if let Some(target) = node.redirect() {
                next.arguments.clear();
                next.executor = None;
                let mut parse = self.parse_nodes(target, reader, next, offered);
                parse.errors.extend(errors);
                return parse;
            }
            potentials.push(self.parse_nodes(id, reader, next, offered));
        }

        let length = reader.input().len();
        potentials.sort_by_key(|parse| {
            (
                parse.cursor < length,
                Reverse(parse.cursor),
                parse.context.executor.is_none(),
            )
        });
        let mut potentials = potentials.into_iter();
        let Some(mut best) = potentials.next() else {
            return Parse {
                context,
                cursor: start,
                errors,
            };
        };
        best.errors.extend(errors);
        best.errors
            .extend(potentials.flat_map(|parse| parse.errors));
        best
    }
}
