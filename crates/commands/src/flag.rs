use crate::reader::{Reader, SyntaxError};
use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Flag {
    name: &'static str,
    short: Option<char>,
    description: &'static str,
    value_name: Option<&'static str>,
}

impl Flag {
    pub const fn switch(
        name: &'static str,
        short: Option<char>,
        description: &'static str,
    ) -> Self {
        Self {
            name,
            short,
            description,
            value_name: None,
        }
    }

    /// No short form, so that a group like `-io` never takes a value.
    pub const fn valued(
        name: &'static str,
        value_name: &'static str,
        description: &'static str,
    ) -> Self {
        Self {
            name,
            short: None,
            description,
            value_name: Some(value_name),
        }
    }

    pub fn name(&self) -> &'static str {
        self.name
    }

    pub fn short(&self) -> Option<char> {
        self.short
    }

    pub fn description(&self) -> &'static str {
        self.description
    }

    pub fn value_name(&self) -> Option<&'static str> {
        self.value_name
    }
}

impl fmt::Display for Flag {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if let Some(short) = self.short {
            write!(f, "-{short}, ")?;
        }
        write!(f, "--{}", self.name)?;
        if let Some(value_name) = self.value_name {
            write!(f, " <{value_name}>")?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FlagValue {
    Switch(bool),
    Text(String),
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct FlagValues(Vec<(Flag, FlagValue)>);

impl FlagValues {
    pub fn set(&mut self, flag: Flag, value: FlagValue) {
        match self.0.iter_mut().find(|(known, _)| known.name == flag.name) {
            Some(entry) => entry.1 = value,
            None => self.0.push((flag, value)),
        }
    }

    pub fn extend(&mut self, overrides: FlagValues) {
        for (flag, value) in overrides.0 {
            self.set(flag, value);
        }
    }

    pub fn get(&self, name: &str) -> Option<&FlagValue> {
        self.0
            .iter()
            .find(|(flag, _)| flag.name == name)
            .map(|(_, value)| value)
    }

    pub fn is_on(&self, flag: &Flag) -> bool {
        self.get(flag.name) == Some(&FlagValue::Switch(true))
    }

    pub fn text(&self, flag: &Flag) -> Option<&str> {
        match self.get(flag.name) {
            Some(FlagValue::Text(text)) => Some(text),
            _ => None,
        }
    }

    pub fn names(&self) -> impl Iterator<Item = &'static str> + '_ {
        self.0.iter().map(|(flag, _)| flag.name)
    }
}

impl fmt::Display for FlagValues {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let on = |flag: &&Flag| self.is_on(flag);
        let flags = || self.0.iter().map(|(flag, _)| flag);
        let shorts: String = flags().filter(on).filter_map(|flag| flag.short).collect();
        let mut words = Vec::new();
        if !shorts.is_empty() {
            words.push(format!("-{shorts}"));
        }
        for flag in flags().filter(on).filter(|flag| flag.short.is_none()) {
            words.push(format!("--{}", flag.name));
        }
        for (flag, value) in &self.0 {
            if let FlagValue::Text(text) = value {
                words.push(format!("--{} {}", flag.name, quoted(text)));
            }
        }
        write!(f, "{}", words.join(" "))
    }
}

pub fn parse(reader: &mut Reader<'_>, flags: &[Flag]) -> Result<FlagValues, SyntaxError> {
    let mut values = FlagValues::default();
    reader.skip_whitespace();
    while reader.can_read() {
        let token = reader.token()?;
        if let Some(long) = token.strip_prefix("--") {
            let (name, given) = match long.split_once('=') {
                Some((name, given)) => (name, Some(given)),
                None => (long, None),
            };
            let flag = flags
                .iter()
                .find(|flag| flag.name == name)
                .ok_or_else(|| reader.error(format!("Unknown flag: --{name}")))?;
            let value = match (flag.value_name, given) {
                (None, None | Some("true")) => FlagValue::Switch(true),
                (None, Some("false")) => FlagValue::Switch(false),
                (None, Some(given)) => {
                    return Err(reader.error(format!(
                        "Expected true or false for --{name}, found {given}"
                    )));
                }
                (Some(_), Some("")) => {
                    return Err(reader.error(format!("Expected a value for --{name}")));
                }
                (Some(_), Some(given)) => FlagValue::Text(given.to_owned()),
                (Some(_), None) => {
                    reader.skip_whitespace();
                    if !reader.can_read() {
                        return Err(reader.error(format!("Expected a value for --{name}")));
                    }
                    FlagValue::Text(reader.custom_string()?)
                }
            };
            values.set(*flag, value);
        } else if let Some(shorts) = token.strip_prefix('-').filter(|shorts| !shorts.is_empty()) {
            for short in shorts.chars() {
                let flag = find_short(flags, short)
                    .ok_or_else(|| reader.error(format!("Unknown flag: -{short}")))?;
                values.set(*flag, FlagValue::Switch(true));
            }
        } else {
            return Err(reader.error(format!("Expected a flag, found {token}")));
        }
        reader.skip_whitespace();
    }
    Ok(values)
}

fn quoted(text: &str) -> String {
    if !text.is_empty() && !text.contains(char::is_whitespace) && !text.starts_with(['"', '\'']) {
        return text.to_owned();
    }
    format!("\"{}\"", text.replace('\\', "\\\\").replace('"', "\\\""))
}

fn find_short(flags: &[Flag], short: char) -> Option<&Flag> {
    let with = |short| flags.iter().find(|flag| flag.short == Some(short));
    with(short).or_else(|| with(short.to_ascii_lowercase()))
}
