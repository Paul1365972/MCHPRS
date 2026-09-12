use super::argument::Parameter;
use super::graph::{Graph, Node, NodeId, NodeKind, RegistrationError};
use std::fmt;

pub enum Syntax {
    Argument(String, Parameter),
    Sequence(Vec<Syntax>),
    Optional(Vec<Syntax>),
    Alternatives(Vec<Syntax>),
}

impl Syntax {
    pub fn register(
        self,
        graph: &mut Graph,
        parent: NodeId,
    ) -> Result<Vec<NodeId>, RegistrationError> {
        let mut paths = self.paths();
        // Of two readings that use up the input, the parser and the client keep the first.
        paths.sort_by_key(Vec::len);
        let mut registered = Vec::new();
        for path in paths {
            let mut terminal = parent;
            for (index, &(name, parameter)) in path.iter().enumerate() {
                if path[..index].iter().any(|&(previous, _)| previous == name) {
                    return Err(RegistrationError::RepeatedArgument(name.to_owned()));
                }
                terminal = graph.insert(
                    terminal,
                    Node::new(
                        NodeKind::Argument {
                            name: name.to_owned(),
                            parameter: parameter.clone(),
                        },
                        None,
                    ),
                )?;
            }
            registered.push(terminal);
        }

        Ok(registered)
    }

    fn paths(&self) -> Vec<Vec<(&str, &Parameter)>> {
        match self {
            Self::Argument(name, parameter) => vec![vec![(name, parameter)]],
            Self::Sequence(items) | Self::Optional(items) => {
                let mut paths = vec![Vec::new()];
                for item in items {
                    let suffixes = item.paths();
                    paths = paths
                        .into_iter()
                        .flat_map(|prefix| {
                            suffixes
                                .iter()
                                .map(move |suffix| prefix.iter().chain(suffix).copied().collect())
                        })
                        .collect();
                }
                if matches!(self, Self::Optional(_)) {
                    paths.push(Vec::new());
                }
                paths
            }
            Self::Alternatives(items) => items.iter().flat_map(Self::paths).collect(),
        }
    }
}

impl fmt::Display for Syntax {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (items, opening, separator, closing) = match self {
            Self::Argument(name, _) => return write!(formatter, "<{name}>"),
            Self::Sequence(items) => (items, "", " ", ""),
            Self::Optional(items) => (items, "[", " ", "]"),
            Self::Alternatives(items) => (items, "(", " | ", ")"),
        };

        formatter.write_str(opening)?;
        for (index, item) in items.iter().enumerate() {
            if index > 0 {
                formatter.write_str(separator)?;
            }
            write!(formatter, "{item}")?;
        }
        formatter.write_str(closing)
    }
}
