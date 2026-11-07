use crate::ast::{self, Component, Output};
use crate::lexer::{ComponentType, Lexer, Token, TokenKind};
use crate::{Error, Input, Module, Options, Test};
use itertools::Itertools;
use mchprs_blocks::blocks::{ComparatorMode, Instrument};
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::circuit::Circuit;
use mchprs_redpiler::graph::{Link, LinkKind, NodeKind, NodeState};
use rustc_hash::FxHashMap;
use std::iter::Peekable;
use std::vec;

enum Global {
    Circuit(usize),
    Schematic(usize),
    Test,
}

struct NamedOutput {
    target: String,
    offset: usize,
    link: Link,
}

pub(crate) struct Parser<'a> {
    source: &'a str,
    tokens: Peekable<vec::IntoIter<Token>>,
    globals: FxHashMap<String, Global>,
    test_args: Option<Options>,
    circuits: Vec<ast::Circuit>,
    schematics: Vec<ast::Schematic>,
    tests: Vec<Test>,
}

impl<'a> Parser<'a> {
    pub(crate) fn parse(source: &'a str) -> Result<Module, Error> {
        let mut parser = Parser {
            source,
            tokens: Lexer::lex(source)?.into_iter().peekable(),
            globals: FxHashMap::default(),
            test_args: None,
            circuits: Vec::new(),
            schematics: Vec::new(),
            tests: Vec::new(),
        };
        while parser.tokens.peek().is_some() {
            let keyword = parser.expect_token(&[
                TokenKind::Circuit,
                TokenKind::Schematic,
                TokenKind::Test,
                TokenKind::TestArgs,
            ])?;
            match keyword.kind {
                TokenKind::Circuit => {
                    let circuit = parser.parse_circuit(keyword.offset)?;
                    let global = Global::Circuit(parser.circuits.len());
                    parser.globals.insert(circuit.name.clone(), global);
                    parser.circuits.push(circuit);
                }
                TokenKind::Schematic => {
                    let schematic = parser.parse_schematic()?;
                    let global = Global::Schematic(parser.schematics.len());
                    parser.globals.insert(schematic.name.clone(), global);
                    parser.schematics.push(schematic);
                }
                TokenKind::Test => parser.parse_test(keyword.offset)?,
                TokenKind::TestArgs => parser.parse_test_args(keyword.offset)?,
                _ => unreachable!(),
            }
        }
        Ok(Module {
            source: source.to_owned(),
            circuits: parser.circuits,
            tests: parser.tests,
        })
    }

    fn error(&self, offset: usize, message: impl Into<String>) -> Error {
        Error::at(self.source, offset, message)
    }

    fn parse_global_name(&mut self) -> Result<String, Error> {
        let token = self.expect_token_with(
            |kind| matches!(kind, TokenKind::GlobalValue(_)),
            &[TokenKind::GlobalValue(String::new())],
        )?;
        let TokenKind::GlobalValue(name) = token.kind else {
            unreachable!()
        };
        if self.globals.contains_key(&name) {
            return Err(self.error(token.offset, format!("duplicate name @{name}")));
        }
        Ok(name)
    }

    fn parse_schematic(&mut self) -> Result<ast::Schematic, Error> {
        let name = self.parse_global_name()?;
        let (_, path) = self.expect_string()?;
        Ok(ast::Schematic { name, path })
    }

    fn parse_circuit(&mut self, keyword_offset: usize) -> Result<ast::Circuit, Error> {
        let name = self.parse_global_name()?;
        self.expect_token(&[TokenKind::LeftCurlyBrace])?;

        let mut components = Vec::new();
        let mut offsets = Vec::new();
        let mut named_outputs = Vec::new();
        let mut indices = FxHashMap::default();
        let end = loop {
            let token = self.expect_token_with(
                |kind| matches!(kind, TokenKind::Value(_) | TokenKind::RightCurlyBrace),
                &[TokenKind::Value(String::new()), TokenKind::RightCurlyBrace],
            )?;
            let TokenKind::Value(value) = token.kind else {
                break token.offset + 1;
            };
            if indices.insert(value.clone(), components.len()).is_some() {
                return Err(self.error(token.offset, format!("duplicate value %{value}")));
            }
            let (component, outputs) = self.parse_component(value)?;
            components.push(component);
            offsets.push(token.offset);
            named_outputs.push(outputs);
        };

        for (component, outputs) in components.iter_mut().zip(named_outputs) {
            for output in outputs {
                let Some(&target) = indices.get(&output.target) else {
                    let message = format!("unknown value %{}", output.target);
                    return Err(self.error(output.offset, message));
                };
                component.outputs.push(Output {
                    target,
                    link: output.link,
                });
            }
        }

        let circuit = ast::Circuit {
            name,
            components,
            span: keyword_offset..end,
        };
        if let Err(invalid) = Circuit::check(&circuit.build_graph(|_| true)) {
            return Err(match invalid.node() {
                Some(index) => {
                    let name = &circuit.components[index].name;
                    self.error(offsets[index], format!("%{name} {}", invalid.problem()))
                }
                None => {
                    let message =
                        format!("invalid circuit @{}: {}", circuit.name, invalid.problem());
                    self.error(keyword_offset, message)
                }
            });
        }
        Ok(circuit)
    }

    fn parse_component(&mut self, name: String) -> Result<(Component, Vec<NamedOutput>), Error> {
        self.expect_token(&[TokenKind::Equals])?;
        let component_type = self.expect_token_with(
            |kind| matches!(kind, TokenKind::ComponentType(_)),
            &[TokenKind::ComponentType(ComponentType::Wire)],
        )?;
        let TokenKind::ComponentType(component_type) = component_type.kind else {
            unreachable!()
        };
        let (kind, state) = match component_type {
            ComponentType::Repeater => {
                let (delay_offset, delay) = self.expect_int()?;
                let Ok(delay) = u8::try_from(delay) else {
                    return Err(self.error(delay_offset, "repeater delay out of range"));
                };
                self.expect_token(&[TokenKind::Comma])?;
                let prioritized = self.expect_bool()?;
                self.expect_token(&[TokenKind::Comma])?;
                let locked = self.expect_bool()?;
                self.expect_token(&[TokenKind::Comma])?;
                let powered = self.expect_bool()?;
                let kind = NodeKind::Repeater { delay, prioritized };
                (kind, NodeState::repeater(powered, locked))
            }
            ComponentType::Comparator => {
                let mode = self.expect_token_with(
                    |kind| matches!(kind, TokenKind::ComparatorMode(_)),
                    &[TokenKind::ComparatorMode(ComparatorMode::Subtract)],
                )?;
                let TokenKind::ComparatorMode(mode) = mode.kind else {
                    unreachable!()
                };
                self.expect_token(&[TokenKind::Comma])?;
                let far_input = self.parse_comparator_far_input()?;
                self.expect_token(&[TokenKind::Comma])?;
                let prioritized = self.expect_bool()?;
                self.expect_token(&[TokenKind::Comma])?;
                let power = self.expect_power()?;
                let kind = NodeKind::Comparator {
                    mode,
                    far_input,
                    prioritized,
                };
                (kind, NodeState::from_power(power))
            }
            ComponentType::Button => (NodeKind::Button, self.expect_powered()?),
            ComponentType::Torch => (NodeKind::Torch, self.expect_powered()?),
            ComponentType::Lamp => (NodeKind::Lamp, self.expect_powered()?),
            ComponentType::Trapdoor => (NodeKind::Trapdoor, self.expect_powered()?),
            ComponentType::Lever => (NodeKind::Lever, self.expect_powered()?),
            ComponentType::PressurePlate => (NodeKind::PressurePlate, self.expect_powered()?),
            ComponentType::Wire => (NodeKind::Wire, NodeState::from_power(self.expect_power()?)),
            ComponentType::Constant => {
                let mut state = NodeState::from_power(self.expect_power()?);
                if self
                    .tokens
                    .next_if(|token| token.kind == TokenKind::Comma)
                    .is_some()
                {
                    state.repeater_locked = self.expect_bool()?;
                }
                (NodeKind::Constant, state)
            }
            ComponentType::NoteBlock => {
                let instrument = self.expect_token_with(
                    |kind| matches!(kind, TokenKind::Instrument(_)),
                    &[TokenKind::Instrument(Instrument::Harp)],
                )?;
                let TokenKind::Instrument(instrument) = instrument.kind else {
                    unreachable!()
                };
                self.expect_token(&[TokenKind::Comma])?;
                let (note_offset, note) = self.expect_int()?;
                let Ok(note) = u8::try_from(note) else {
                    return Err(self.error(note_offset, "note out of range"));
                };
                self.expect_token(&[TokenKind::Comma])?;
                let powered = self.expect_bool()?;
                let kind = NodeKind::NoteBlock { instrument, note };
                (kind, NodeState::from_powered(powered))
            }
        };
        let port = match self
            .tokens
            .next_if(|token| matches!(token.kind, TokenKind::Port(_)))
        {
            Some(Token {
                kind: TokenKind::Port(port),
                ..
            }) => port,
            _ => kind.conventional_role(),
        };
        let outputs = self.parse_outputs()?;
        let component = Component {
            name,
            kind,
            state,
            port,
            outputs: Vec::new(),
        };
        Ok((component, outputs))
    }

    fn parse_comparator_far_input(&mut self) -> Result<Option<SignalStrength>, Error> {
        let token = self.expect_token_with(
            |kind| matches!(kind, TokenKind::Int(_) | TokenKind::None),
            &[TokenKind::Int(0), TokenKind::None],
        )?;
        match token.kind {
            TokenKind::Int(power) => SignalStrength::try_from(power)
                .map(Some)
                .map_err(|error| self.error(token.offset, error.to_string())),
            _ => Ok(None),
        }
    }

    fn parse_test(&mut self, keyword_offset: usize) -> Result<(), Error> {
        self.expect_token(&[TokenKind::LeftParens])?;
        let input_token = self.expect_token_with(
            |kind| matches!(kind, TokenKind::GlobalValue(_)),
            &[TokenKind::GlobalValue(String::new())],
        )?;
        let TokenKind::GlobalValue(input_name) = input_token.kind else {
            unreachable!()
        };
        let input = match self.globals.get(&input_name) {
            Some(&Global::Circuit(index)) => Input::Circuit(self.circuits[index].clone()),
            Some(&Global::Schematic(index)) => Input::Schematic(self.schematics[index].clone()),
            Some(Global::Test) => {
                let message = format!("@{input_name} is a test, not a circuit or schematic");
                return Err(self.error(input_token.offset, message));
            }
            None => {
                let message = format!("unknown circuit or schematic @{input_name}");
                return Err(self.error(input_token.offset, message));
            }
        };
        let parens_or_comma = self.expect_token(&[TokenKind::RightParens, TokenKind::Comma])?;
        let options = if parens_or_comma.kind == TokenKind::Comma {
            let options = self.expect_options()?;
            self.expect_token(&[TokenKind::RightParens])?;
            options
        } else {
            self.test_args
                .clone()
                .ok_or_else(|| self.error(keyword_offset, "missing test_args"))?
        };
        let circuit_keyword = self.expect_token(&[TokenKind::Circuit])?;
        let expected = self.parse_circuit(circuit_keyword.offset)?;
        self.globals.insert(expected.name.clone(), Global::Test);
        self.tests.push(Test {
            name: expected.name.clone(),
            input,
            options,
            expected,
        });
        Ok(())
    }

    fn parse_test_args(&mut self, keyword_offset: usize) -> Result<(), Error> {
        if self.test_args.is_some() {
            return Err(self.error(keyword_offset, "duplicate test_args"));
        }
        self.test_args = Some(self.expect_options()?);
        Ok(())
    }

    fn expect_options(&mut self) -> Result<Options, Error> {
        let (offset, arguments) = self.expect_string()?;
        Options::parse(&arguments).map_err(|message| self.error(offset, message))
    }

    fn parse_outputs(&mut self) -> Result<Vec<NamedOutput>, Error> {
        if self
            .tokens
            .next_if(|token| token.kind == TokenKind::Arrow)
            .is_none()
        {
            return Ok(Vec::new());
        }
        self.expect_token(&[TokenKind::LeftBracket])?;
        let mut outputs = Vec::new();
        loop {
            outputs.push(self.parse_output()?);
            let comma_or_bracket =
                self.expect_token(&[TokenKind::Comma, TokenKind::RightBracket])?;
            if comma_or_bracket.kind == TokenKind::RightBracket {
                return Ok(outputs);
            }
        }
    }

    fn parse_output(&mut self) -> Result<NamedOutput, Error> {
        let kind = match self.tokens.next_if(|token| token.kind == TokenKind::Side) {
            Some(_) => LinkKind::Side,
            None => LinkKind::Default,
        };
        let target = self.expect_token_with(
            |kind| matches!(kind, TokenKind::Value(_)),
            &[TokenKind::Side, TokenKind::Value(String::new())],
        )?;
        let TokenKind::Value(target_name) = target.kind else {
            unreachable!()
        };
        self.expect_token(&[TokenKind::Colon])?;
        let (weight_offset, weight) = self.expect_int()?;
        let weight = u8::try_from(weight)
            .ok()
            .filter(|&weight| weight < 15)
            .ok_or_else(|| {
                self.error(
                    weight_offset,
                    format!("link weight {weight} is outside 0..=14"),
                )
            })?;
        Ok(NamedOutput {
            target: target_name,
            offset: target.offset,
            link: Link::new(kind, weight),
        })
    }

    fn expect_power(&mut self) -> Result<SignalStrength, Error> {
        let (offset, power) = self.expect_int()?;
        SignalStrength::try_from(power).map_err(|error| self.error(offset, error.to_string()))
    }

    fn expect_powered(&mut self) -> Result<NodeState, Error> {
        Ok(NodeState::from_powered(self.expect_bool()?))
    }

    fn expect_int(&mut self) -> Result<(usize, u32), Error> {
        let token = self.expect_token_with(
            |kind| matches!(kind, TokenKind::Int(_)),
            &[TokenKind::Int(0)],
        )?;
        let TokenKind::Int(int) = token.kind else {
            unreachable!()
        };
        Ok((token.offset, int))
    }

    fn expect_bool(&mut self) -> Result<bool, Error> {
        let token = self.expect_token_with(
            |kind| matches!(kind, TokenKind::Bool(_)),
            &[TokenKind::Bool(false)],
        )?;
        let TokenKind::Bool(bool) = token.kind else {
            unreachable!()
        };
        Ok(bool)
    }

    fn expect_string(&mut self) -> Result<(usize, String), Error> {
        let token = self.expect_token_with(
            |kind| matches!(kind, TokenKind::String(_)),
            &[TokenKind::String(String::new())],
        )?;
        let TokenKind::String(string) = token.kind else {
            unreachable!()
        };
        Ok((token.offset, string))
    }

    fn expect_token(&mut self, expected: &[TokenKind]) -> Result<Token, Error> {
        self.expect_token_with(|kind| expected.contains(kind), expected)
    }

    fn expect_token_with(
        &mut self,
        accepts: impl Fn(&TokenKind) -> bool,
        expected: &[TokenKind],
    ) -> Result<Token, Error> {
        let expected_names = || expected.iter().map(TokenKind::friendly_name).join(", ");
        let Some(token) = self.tokens.next() else {
            let message = format!("reached end of file, expected one of: {}", expected_names());
            return Err(self.error(self.source.len(), message));
        };
        if !accepts(&token.kind) {
            let message = format!(
                "found {}, expected one of: {}",
                token.kind.friendly_name(),
                expected_names()
            );
            return Err(self.error(token.offset, message));
        }
        Ok(token)
    }
}
