use crate::expect::Expect;
use crate::reader::{Reader, SyntaxError};
use std::fmt;
use std::iter;
use std::mem;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Flag {
    name: &'static str,
    short: Option<char>,
    description: &'static str,
    value_name: Option<&'static str>,
    exclusive_group: Option<&'static str>,
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
            exclusive_group: None,
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
            exclusive_group: None,
        }
    }

    pub const fn excluding_others_of(self, group: &'static str) -> Self {
        Self {
            exclusive_group: Some(group),
            ..self
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

    fn excludes(&self, other: &Flag) -> bool {
        self.name != other.name
            && self.exclusive_group.is_some()
            && self.exclusive_group == other.exclusive_group
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
enum FlagValue {
    Switch(bool),
    Text(String),
}

impl FlagValue {
    fn is_off(&self) -> bool {
        *self == FlagValue::Switch(false)
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct FlagValues(Vec<(Flag, FlagValue)>);

impl FlagValues {
    pub fn parse(text: &str, accepted: &[Flag]) -> Result<FlagValues, InvalidFlags> {
        let mut expect = Expect::executing();
        parse(&mut Reader::new(text), accepted, &mut expect).map_err(|error| InvalidFlags {
            text: text.to_owned(),
            error,
        })
    }

    pub fn take(&mut self, flags: &[Flag]) -> FlagValues {
        let is_taken =
            |(flag, _): &(Flag, FlagValue)| flags.iter().any(|taken| taken.name == flag.name);
        let (taken, kept) = mem::take(&mut self.0).into_iter().partition(is_taken);
        self.0 = kept;
        FlagValues(taken)
    }

    pub fn extend(&mut self, overrides: FlagValues) {
        for (flag, value) in overrides.0 {
            self.set(flag, value);
        }
    }

    pub fn contains(&self, flag: &Flag) -> bool {
        self.value(flag).is_some()
    }

    pub fn flags(&self) -> impl Iterator<Item = &Flag> {
        self.0.iter().map(|(flag, _)| flag)
    }

    pub fn is_on(&self, flag: &Flag) -> bool {
        self.value(flag) == Some(&FlagValue::Switch(true))
    }

    pub fn text(&self, flag: &Flag) -> Option<&str> {
        match self.value(flag) {
            Some(FlagValue::Text(text)) => Some(text),
            _ => None,
        }
    }

    fn value(&self, flag: &Flag) -> Option<&FlagValue> {
        self.0
            .iter()
            .find(|(known, _)| known.name == flag.name)
            .map(|(_, value)| value)
    }

    fn set_once(
        &mut self,
        flag: Flag,
        value: FlagValue,
        start: usize,
        expect: &mut Expect,
    ) -> Result<(), SyntaxError> {
        if !value.is_off()
            && let Some(given) = self.excluding(&flag)
        {
            return Err(expect.fail_recognized(
                start,
                format!("--{} can't be used with --{}", flag.name, given.name),
            ));
        }
        self.set(flag, value);
        Ok(())
    }

    fn excluding(&self, flag: &Flag) -> Option<&Flag> {
        self.0
            .iter()
            .filter(|(_, value)| !value.is_off())
            .map(|(given, _)| given)
            .find(|given| flag.excludes(given))
    }

    fn set(&mut self, flag: Flag, value: FlagValue) {
        if !value.is_off() {
            self.0.retain(|(known, _)| !flag.excludes(known));
        }
        match self.0.iter_mut().find(|(known, _)| known.name == flag.name) {
            Some(entry) => entry.1 = value,
            None => self.0.push((flag, value)),
        }
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

#[derive(Debug, Clone, thiserror::Error)]
#[error("{}", .error.contextual(.text))]
pub struct InvalidFlags {
    text: String,
    error: SyntaxError,
}

pub fn parse(
    reader: &mut Reader<'_>,
    flags: &[Flag],
    expect: &mut Expect,
) -> Result<FlagValues, SyntaxError> {
    let mut values = FlagValues::default();
    reader.skip_whitespace();
    loop {
        let start = reader.cursor();
        let token = reader.take_while(|ch| !ch.is_whitespace() && ch != '=');
        expect.offer_with_tooltips(reader, start, || {
            flags
                .iter()
                .filter(|flag| !values.contains(flag) && values.excluding(flag).is_none())
                .flat_map(|flag| {
                    let short = flag.short.map(|short| format!("-{short}"));
                    iter::once(format!("--{}", flag.name))
                        .chain(short)
                        .map(|name| (name, Some(flag.description)))
                })
        });
        if token.is_empty() && !reader.can_read() {
            return Ok(values);
        }
        if let Some(name) = token.strip_prefix("--") {
            let flag = flags
                .iter()
                .find(|flag| flag.name == name)
                .ok_or_else(|| expect.fail_recognized(start, format!("Unknown flag: --{name}")))?;
            let value = if flag.value_name.is_some() {
                if reader.peek() == Some('=') {
                    reader.read();
                } else {
                    reader.skip_whitespace();
                }
                if reader.peek().is_none_or(char::is_whitespace) {
                    return Err(
                        expect.fail_recognized(start, format!("Expected a value for --{name}"))
                    );
                }
                FlagValue::Text(reader.custom_string()?)
            } else {
                FlagValue::Switch(switch_state(reader, expect, start)?)
            };
            values.set_once(*flag, value, start, expect)?;
        } else if let Some(shorts) = token.strip_prefix('-').filter(|shorts| !shorts.is_empty()) {
            let group = shorts
                .char_indices()
                .map(|(index, short)| {
                    let at = start + 1 + index;
                    find_short(flags, short)
                        .map(|flag| (at, *flag))
                        .ok_or_else(|| {
                            expect.fail_recognized(at, format!("Unknown flag: -{short}"))
                        })
                })
                .collect::<Result<Vec<_>, _>>()?;
            let state = FlagValue::Switch(switch_state(reader, expect, start)?);
            for (at, flag) in group {
                values.set_once(flag, state.clone(), at, expect)?;
            }
        } else {
            reader.take_token();
            let found = &reader.input()[start..reader.cursor()];
            return Err(expect.fail(start, format!("Expected a flag, found {found}")));
        }
        if !reader.can_read() {
            return Ok(values);
        }
        reader.skip_whitespace();
    }
}

fn switch_state(
    reader: &mut Reader<'_>,
    expect: &mut Expect,
    start: usize,
) -> Result<bool, SyntaxError> {
    if reader.peek() != Some('=') {
        return Ok(true);
    }
    let switch = &reader.input()[start..reader.cursor()];
    reader.read();
    match reader.take_token() {
        "true" => Ok(true),
        "false" => Ok(false),
        given => Err(expect.fail_recognized(
            start,
            format!("Expected true or false for {switch}, found {given}"),
        )),
    }
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
