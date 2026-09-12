use std::fmt;

/// A block state written as `namespace:path[name=value,…]`, not yet resolved to a block
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BlockStateSpec<'a> {
    pub namespace: &'a str,
    pub path: &'a str,
    pub properties: Vec<(&'a str, &'a str)>,
}

impl<'a> BlockStateSpec<'a> {
    pub fn parse(input: &'a str) -> Result<BlockStateSpec<'a>, BlockStateSyntaxError> {
        let (identifier, properties) = match input.split_once('[') {
            Some((identifier, rest)) => {
                let list = rest
                    .strip_suffix(']')
                    .ok_or(BlockStateSyntaxError::UnclosedBracket)?;
                (identifier, parse_properties(list)?)
            }
            None => (input, Vec::new()),
        };
        if identifier.is_empty() {
            return Err(BlockStateSyntaxError::MissingName);
        }
        let (namespace, path) = identifier
            .split_once(':')
            .unwrap_or(("minecraft", identifier));
        Ok(BlockStateSpec {
            namespace,
            path,
            properties,
        })
    }

    pub fn name(&self) -> String {
        format!("{}:{}", self.namespace, self.path)
    }
}

/// Parses the `name=value,…` list between a block state's brackets.
pub fn parse_properties(list: &str) -> Result<Vec<(&str, &str)>, BlockStateSyntaxError> {
    let mut properties = Vec::new();
    if list.is_empty() {
        return Ok(properties);
    }
    for property in list.split(',') {
        let (name, value) = property
            .split_once('=')
            .filter(|(name, value)| !name.is_empty() && !value.is_empty() && !value.contains('='))
            .ok_or_else(|| BlockStateSyntaxError::InvalidProperty(property.to_owned()))?;
        if properties.iter().any(|&(existing, _)| existing == name) {
            return Err(BlockStateSyntaxError::DuplicateProperty(name.to_owned()));
        }
        properties.push((name, value));
    }
    Ok(properties)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BlockStateSyntaxError {
    MissingName,
    UnclosedBracket,
    InvalidProperty(String),
    DuplicateProperty(String),
}

impl fmt::Display for BlockStateSyntaxError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BlockStateSyntaxError::MissingName => f.write_str("Missing block name"),
            BlockStateSyntaxError::UnclosedBracket => {
                f.write_str("Unclosed bracket in block states")
            }
            BlockStateSyntaxError::InvalidProperty(property) => {
                write!(f, "Invalid block state: {property}")
            }
            BlockStateSyntaxError::DuplicateProperty(name) => {
                write!(f, "Duplicate block property: {name}")
            }
        }
    }
}

impl std::error::Error for BlockStateSyntaxError {}
