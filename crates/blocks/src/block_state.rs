//! Block states as commands write them, without data tags: <https://minecraft.wiki/w/Argument_types#minecraft:block_state>

use crate::blocks::{Block, PropertyDefinition};
use mchprs_syntax::expect::Expect;
use mchprs_syntax::reader::{Reader, SyntaxError};
use std::fmt::{self, Write as _};
use std::iter;

impl fmt::Display for Block {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.get_name())?;
        let properties = self.properties();
        if properties.is_empty() {
            return Ok(());
        }
        f.write_char('[')?;
        for (index, (name, value)) in properties.iter().enumerate() {
            if index > 0 {
                f.write_char(',')?;
            }
            write!(f, "{name}={value}")?;
        }
        f.write_char(']')
    }
}

#[derive(Clone, Debug, Default)]
pub struct StateList(Vec<(&'static str, &'static str)>);

impl StateList {
    pub fn matches(&self, block: Block, lenient: bool) -> bool {
        let properties = block.properties();
        self.0
            .iter()
            .all(|&(name, value)| match properties.get(name) {
                Some(actual) => actual == value,
                None => lenient,
            })
    }

    pub fn applied_to(&self, mut block: Block) -> Block {
        if !self.0.is_empty() {
            block.set_properties(self.0.iter().copied().collect());
        }
        block
    }

    pub fn states(&self) -> impl Iterator<Item = (&'static str, &'static str)> + '_ {
        self.0.iter().copied()
    }
}

pub fn block_state(
    reader: &mut Reader<'_>,
    expect: &mut Expect,
) -> Result<(Block, StateList), SyntaxError> {
    let start = reader.cursor();
    let name = reader.take_while(is_name_char);
    expect.offer(reader, start, || block_names(!name.is_empty()));
    if name.is_empty() {
        return Err(expect.fail(start, "Expected a block"));
    }
    let block =
        block_named(name).ok_or_else(|| expect.fail(start, format!("Unknown block: {name}")))?;
    if !block.properties().is_empty() {
        expect.offer(reader, start, || [format!("{name}[")]);
    }
    if reader.peek() != Some('[') {
        return Ok((block, StateList::default()));
    }
    let states = state_list(reader, expect, Some(block))?;
    Ok((block, states))
}

/// An unknown block or property value gives `None`; a property the block lacks is skipped.
pub fn stored(text: &str) -> Result<Option<Block>, SyntaxError> {
    let mut reader = Reader::new(text);
    let expect = &mut Expect::executing();
    let name = reader.take_while(is_name_char);
    let (namespace, path) = name.split_once(':').unwrap_or(("minecraft", name));
    if namespace.is_empty() || namespace.contains('/') || path.is_empty() || path.contains(':') {
        return Err(expect.fail(0, "Expected a block"));
    }
    let block = block_named(name);
    let mut states = Some(StateList::default());
    if reader.peek() == Some('[') {
        states = read_states(&mut reader, expect, block, true)?;
    }
    if reader.peek().is_some() {
        return Err(expect.fail(reader.cursor(), "Expected the end of the block state"));
    }
    Ok(block
        .zip(states)
        .map(|(block, states)| states.applied_to(block)))
}

pub fn state_list(
    reader: &mut Reader<'_>,
    expect: &mut Expect,
    block: Option<Block>,
) -> Result<StateList, SyntaxError> {
    let states = read_states(reader, expect, block, false)?;
    Ok(states.expect("strict reading fails on an invalid value"))
}

pub fn state_id(reader: &mut Reader<'_>, expect: &mut Expect) -> Result<Block, SyntaxError> {
    let start = reader.cursor();
    let text = reader.take_while(|ch| ch != ',' && !ch.is_whitespace());
    let id = text
        .parse::<u32>()
        .map_err(|_| expect.fail(start, format!("Invalid block state ID: ={text}")))?;
    let block = Block::from_id(id);
    if block.get_id() != id {
        return Err(expect.fail(start, format!("Unsupported block state ID: {id}")));
    }
    Ok(block)
}

fn read_states(
    reader: &mut Reader<'_>,
    expect: &mut Expect,
    block: Option<Block>,
    lenient: bool,
) -> Result<Option<StateList>, SyntaxError> {
    if reader.read() != Some('[') {
        return Err(expect.fail(reader.cursor(), "Expected ["));
    }
    let properties = properties_of(block);
    let mut states: Vec<(&'static str, &'static str)> = Vec::new();
    let mut seen: Vec<&str> = Vec::new();
    let mut valid = true;
    if reader.peek() == Some(']') {
        reader.read();
        return Ok(Some(StateList(states)));
    }
    loop {
        let unused = |states: &[(&str, &str)]| {
            properties
                .iter()
                .filter(|property| states.iter().all(|&(name, _)| name != property.name))
                .collect::<Vec<_>>()
        };
        let start = reader.cursor();
        let name = reader.take_while(is_property_char);
        expect.offer(reader, start, || {
            unused(&states)
                .into_iter()
                .map(|property| format!("{}=", property.name))
        });
        if name.is_empty() {
            return Err(expect.fail(start, "Expected a block property"));
        }
        let property = properties.iter().find(|property| property.name == name);
        if property.is_none() && !lenient {
            return Err(expect.fail(start, unknown_property(block, name)));
        }
        if seen.contains(&name) {
            return Err(expect.fail(start, format!("Duplicate block property: {name}")));
        }
        seen.push(name);
        if reader.read() != Some('=') {
            return Err(expect.fail(start + name.len(), "Expected ="));
        }
        let start = reader.cursor();
        let value = reader.take_while(is_value_char);
        let more = unused(&states).len() > 1;
        expect.offer(reader, start, || {
            let values = property.into_iter().flat_map(|property| property.values);
            values.flat_map(|value| {
                iter::once(format!("{value}]")).chain(more.then(|| format!("{value},")))
            })
        });
        if value.is_empty() {
            return Err(expect.fail(start, format!("Expected a value for {name}")));
        }
        if let Some(property) = property {
            match property
                .values
                .iter()
                .find(|candidate| **candidate == value)
            {
                Some(value) => states.push((property.name, value)),
                None if lenient => valid = false,
                None => {
                    let message = format!("Property {} does not accept {value}", property.name);
                    return Err(expect.fail(start, message));
                }
            }
        }
        let separator = reader.cursor();
        match reader.read() {
            Some(',') => {}
            Some(']') => return Ok(valid.then_some(StateList(states))),
            _ => return Err(expect.fail(separator, "Expected , or ]")),
        }
    }
}

fn is_name_char(ch: char) -> bool {
    is_value_char(ch) || matches!(ch, '/' | ':')
}

fn is_property_char(ch: char) -> bool {
    ch.is_ascii_lowercase() || ch.is_ascii_digit() || ch == '_'
}

fn is_value_char(ch: char) -> bool {
    is_property_char(ch) || matches!(ch, '.' | '-')
}

fn block_names(namespaced: bool) -> impl Iterator<Item = &'static str> {
    Block::names().iter().flat_map(move |name| {
        let short = name.strip_prefix("minecraft:").unwrap_or(name);
        iter::once(short).chain(namespaced.then_some(*name))
    })
}

fn block_named(name: &str) -> Option<Block> {
    if name.contains(':') {
        Block::from_name(name)
    } else {
        Block::from_name(&format!("minecraft:{name}"))
    }
}

fn properties_of(block: Option<Block>) -> Vec<PropertyDefinition> {
    match block {
        Some(block) => block
            .properties()
            .into_keys()
            .filter_map(|name| {
                let values = block.property_values(name)?;
                Some(PropertyDefinition { name, values })
            })
            .collect(),
        None => Block::known_properties().to_vec(),
    }
}

fn unknown_property(block: Option<Block>, name: &str) -> String {
    match block {
        Some(block) => format!("Block {} does not have property {name}", block.get_name()),
        None => format!("Unknown block property: {name}"),
    }
}
