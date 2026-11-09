use super::token::is_name_char;
use super::{Parsed, Parser};
use crate::ast::Component;
use crate::fields::{visit, visit_tick, Fields, Value, KINDS, PORTS};
use mchprs_blocks::block_state::block_state;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::BlockPos;
use mchprs_redpiler::circuit::LINK_WEIGHTS;
use mchprs_redpiler::graph::{Link, LinkKind, NodeState, QueuedTick};
use mchprs_redstone::tick_queue::{PendingTick, TickPriority};
use mchprs_syntax::expect::Expect;
use mchprs_syntax::reader::SyntaxError;
use rustc_hash::FxHashMap;

pub struct NamedOutput {
    pub target: String,
    pub at: usize,
    pub link: Link,
}

pub struct Tick {
    pub tick: PendingTick,
    pub order: Option<usize>,
    pub at: usize,
}

impl<'a> Parser<'a> {
    pub fn component(
        &mut self,
        indices: &mut FxHashMap<String, usize>,
        index: usize,
    ) -> Parsed<(Component, Option<Tick>, Vec<NamedOutput>)> {
        let mut labels = Vec::new();
        loop {
            self.require('%')?;
            let at = self.reader.cursor();
            let name = self.name("a value")?.to_owned();
            if indices.insert(name.clone(), index).is_some() {
                return Err(self.fail(at, format!("duplicate value %{name}")));
            }
            labels.push(name);
            if !self.eat(',') {
                break;
            }
        }
        self.require('=')?;
        self.skip();
        let kind_at = self.reader.cursor();
        let mut kind = self.keyword(KINDS, "a component kind")?;
        let mut state = NodeState::default();
        let entries = self.entries()?;
        let mut fields = FieldParser::new(entries, kind_at);
        visit(&mut kind, &mut state, &mut fields);
        fields.finish()?;

        self.skip();
        let mut ahead = self.reader;
        let word = ahead.take_while(is_name_char);
        let port = match PORTS.iter().find(|(name, _)| *name == word) {
            Some(&(_, port)) => {
                self.reader = ahead;
                port
            }
            None => kind.traits().role,
        };
        let tick = self.tick()?;
        let blocks = self.blocks()?;
        let outputs = self.outputs()?;
        let component = Component {
            labels,
            kind,
            state,
            port,
            pending_tick: tick.as_ref().map(|tick| QueuedTick {
                tick: tick.tick,
                order: tick.order.unwrap_or(0),
            }),
            blocks,
            outputs: Vec::new(),
        };
        Ok((component, tick, outputs))
    }

    fn tick(&mut self) -> Parsed<Option<Tick>> {
        self.skip();
        let at = self.reader.cursor();
        if !self.eat_word("tick") {
            return Ok(None);
        }
        if self.reader.peek() != Some('(') {
            let cursor = self.reader.cursor();
            return Err(self.fail(cursor, "expected '('"));
        }
        let entries = self.entries()?;
        let mut tick = PendingTick {
            delay: 0,
            priority: TickPriority::Normal,
        };
        let mut order = None;
        let mut fields = FieldParser::new(entries, at);
        visit_tick(&mut tick, &mut order, &mut fields);
        fields.finish()?;
        Ok(Some(Tick { tick, order, at }))
    }

    fn blocks(&mut self) -> Parsed<Vec<(BlockPos, u32)>> {
        let mut blocks = Vec::new();
        if !self.eat_word("at") {
            return Ok(blocks);
        }
        self.require('[')?;
        loop {
            self.require('(')?;
            let x = self.integer("x")?;
            self.require(',')?;
            let y = self.integer("y")?;
            self.require(',')?;
            let z = self.integer("z")?;
            self.require(')')?;
            self.skip();
            let (block, states) = block_state(&mut self.reader, &mut Expect::executing())?;
            blocks.push((BlockPos::new(x, y, z), states.applied_to(block).get_id()));
            if self.eat(']') {
                return Ok(blocks);
            }
            self.require(',')?;
        }
    }

    fn outputs(&mut self) -> Parsed<Vec<NamedOutput>> {
        let mut outputs = Vec::new();
        if !self.eat('-') {
            return Ok(outputs);
        }
        self.require('>')?;
        self.require('[')?;
        loop {
            let kind = if self.eat_word("side") {
                LinkKind::Side
            } else {
                LinkKind::Default
            };
            self.require('%')?;
            let at = self.reader.cursor();
            let target = self.name("a value")?.to_owned();
            self.require(':')?;
            self.skip();
            let weight_at = self.reader.cursor();
            let weight: u8 = self.integer("link weight")?;
            if !LINK_WEIGHTS.contains(&weight) {
                let message = format!("link weight {weight} is outside {LINK_WEIGHTS:?}");
                return Err(self.fail(weight_at, message));
            }
            outputs.push(NamedOutput {
                target,
                at,
                link: Link::new(kind, weight),
            });
            if self.eat(']') {
                return Ok(outputs);
            }
            self.require(',')?;
        }
    }

    fn entries(&mut self) -> Parsed<Vec<Entry<'a>>> {
        let mut entries = Vec::new();
        if self.reader.peek() != Some('(') {
            return Ok(entries);
        }
        self.reader.read();
        if self.eat(')') {
            return Ok(entries);
        }
        loop {
            self.skip();
            let at = self.reader.cursor();
            let name = self.name("a field")?;
            let value = if self.eat('=') {
                self.skip();
                let value_at = self.reader.cursor();
                Some((self.name("a value")?, value_at))
            } else {
                None
            };
            entries.push(Entry { name, at, value });
            if self.eat(')') {
                return Ok(entries);
            }
            self.require(',')?;
        }
    }
}

struct Entry<'a> {
    name: &'a str,
    at: usize,
    value: Option<(&'a str, usize)>,
}

struct FieldParser<'a> {
    entries: Vec<(Entry<'a>, bool)>,
    clause: usize,
    error: Option<SyntaxError>,
}

impl<'a> FieldParser<'a> {
    fn new(entries: Vec<Entry<'a>>, clause: usize) -> Self {
        FieldParser {
            entries: entries.into_iter().map(|entry| (entry, false)).collect(),
            clause,
            error: None,
        }
    }

    fn finish(self) -> Result<(), SyntaxError> {
        if let Some(error) = self.error {
            return Err(error);
        }
        match self.entries.iter().find(|(_, used)| !used) {
            Some((entry, _)) => Err(SyntaxError::new(
                entry.at,
                format!("unknown field {}", entry.name),
            )),
            None => Ok(()),
        }
    }

    fn fail(&mut self, at: usize, message: String) {
        self.error.get_or_insert(SyntaxError::new(at, message));
    }

    fn take(&mut self, name: &str) -> Option<Option<(&'a str, usize)>> {
        let (entry, used) = self
            .entries
            .iter_mut()
            .find(|(entry, used)| !used && entry.name == name)?;
        *used = true;
        let value = entry.value;
        if let Some((duplicate, _)) = self
            .entries
            .iter()
            .find(|(entry, used)| !used && entry.name == name)
        {
            let at = duplicate.at;
            self.fail(at, format!("duplicate field {name}"));
        }
        Some(value)
    }

    fn parse<T: Value>(&mut self, name: &str, value: Option<(&str, usize)>) -> Option<T> {
        let Some((text, at)) = value else {
            self.fail(self.clause, format!("{name} needs a value"));
            return None;
        };
        let parsed = text.parse().ok();
        if parsed.is_none() {
            self.fail(at, format!("invalid {name}: {text}"));
        }
        parsed
    }
}

impl Fields for FieldParser<'_> {
    fn value<T: Value>(&mut self, name: &str, value: &mut T, default: Option<T>) {
        match (self.take(name), default) {
            (Some(text), _) => {
                if let Some(parsed) = self.parse(name, text) {
                    *value = parsed;
                }
            }
            (None, Some(default)) => *value = default,
            (None, None) => self.fail(self.clause, format!("missing {name}")),
        }
    }

    fn optional<T: Value>(&mut self, name: &str, value: &mut Option<T>) {
        *value = match self.take(name) {
            Some(text) => self.parse(name, text),
            None => None,
        };
    }

    fn flag(&mut self, name: &str, value: &mut bool) {
        *value = match self.take(name) {
            Some(Some((_, at))) => {
                self.fail(at, format!("{name} takes no value"));
                false
            }
            Some(None) => true,
            None => false,
        };
    }

    fn powered(&mut self, power: &mut SignalStrength) {
        let powered = self.take("powered");
        let strength = self.take("power");
        *power = match (powered, strength) {
            (Some(_), Some(_)) => {
                self.fail(
                    self.clause,
                    "powered and power exclude each other".to_owned(),
                );
                SignalStrength::Zero
            }
            (Some(Some((_, at))), None) => {
                self.fail(at, "powered takes no value".to_owned());
                SignalStrength::Zero
            }
            (Some(None), None) => SignalStrength::Fifteen,
            (None, Some(text)) => self.parse("power", text).unwrap_or(SignalStrength::Zero),
            (None, None) => SignalStrength::Zero,
        };
    }
}
