//! Redstone Intermediate Language: a text form of Redpiler's compile graph, and tests of the
//! passes written in it.

pub mod ast;
mod lexer;
mod parser;
mod print;
mod string_replacer;

use indexmap::IndexMap;
use mchprs_commands::flag::{self, Flag};
use mchprs_commands::reader::Reader;
use mchprs_redpiler::extract::{
    ExtractOptions, ILLEGAL_STATES_OUT, IO_ONLY, WIRE_CROSS_OUT, WIRE_DOT_OUT,
};
use mchprs_redpiler::graph::Graph;
use mchprs_redpiler::passes::{self, Pass};
use parser::Parser;
use std::fmt;
use string_replacer::StringReplacer;

pub use print::{component, Printed};

const PASSES: Flag = Flag::valued("passes", "PASSES", "The passes to run, separated by commas");

static FLAGS: [Flag; 5] = [
    PASSES,
    IO_ONLY,
    WIRE_DOT_OUT,
    ILLEGAL_STATES_OUT,
    WIRE_CROSS_OUT,
];

/// A mistake in a RIL file, at a line and column counted from 1.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Error {
    line: usize,
    column: usize,
    message: String,
}

impl Error {
    fn at(source: &str, offset: usize, message: impl Into<String>) -> Error {
        let before = &source[..offset];
        let line_start = before.rfind('\n').map_or(0, |newline| newline + 1);
        Error {
            line: before.matches('\n').count() + 1,
            column: before[line_start..].chars().count() + 1,
            message: message.into(),
        }
    }

    pub fn line(&self) -> usize {
        self.line
    }

    pub fn column(&self) -> usize {
        self.column
    }

    pub fn message(&self) -> &str {
        &self.message
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}:{}: {}", self.line, self.column, self.message)
    }
}

impl std::error::Error for Error {}

#[derive(Clone)]
pub struct Options {
    pub extract: ExtractOptions,
    pub passes: Vec<&'static dyn Pass>,
}

impl Options {
    pub fn parse(arguments: &str) -> Result<Options, String> {
        let flags = flag::parse(&mut Reader::new(arguments), &FLAGS)
            .map_err(|error| error.contextual(arguments))?;
        let passes = flags
            .text(&PASSES)
            .ok_or_else(|| format!("missing --{}", PASSES.name()))?
            .split(',')
            .map(|name| passes::named(name).ok_or_else(|| format!("unknown pass {name}")))
            .collect::<Result<_, _>>()?;
        Ok(Options {
            extract: ExtractOptions::from_flags(&flags),
            passes,
        })
    }
}

impl fmt::Debug for Options {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let passes: Vec<_> = self.passes.iter().map(|pass| pass.name()).collect();
        f.debug_struct("Options")
            .field("extract", &self.extract)
            .field("passes", &passes)
            .finish()
    }
}

/// A parsed RIL file. Every name in it resolves, and every circuit keeps the circuit invariants.
#[derive(Debug, Clone)]
pub struct Module {
    source: String,
    circuits: Vec<ast::Circuit>,
    tests: Vec<Test>,
}

impl Module {
    pub fn parse(source: &str) -> Result<Module, Error> {
        Parser::parse(source)
    }

    /// The circuits that are not test results, in file order.
    pub fn circuits(&self) -> &[ast::Circuit] {
        &self.circuits
    }

    pub fn circuit(&self, name: &str) -> Option<&ast::Circuit> {
        self.circuits.iter().find(|circuit| circuit.name == name)
    }

    pub fn tests(&self) -> &[Test] {
        &self.tests
    }

    /// The source with the expected circuit of each given test replaced by its result.
    pub fn with_results(&self, results: &[(&Test, &Graph)]) -> String {
        let mut replacements: Vec<_> = results
            .iter()
            .map(|&(test, graph)| {
                let printed = Printed::new(graph, &test.name).to_string();
                (test.expected.span.clone(), printed)
            })
            .collect();
        replacements.sort_by_key(|(span, _)| span.start);
        let mut replacer = StringReplacer::new(&self.source);
        for (span, printed) in replacements {
            replacer.replace_range(span, &printed);
        }
        replacer.finish()
    }
}

#[derive(Debug, Clone)]
pub struct Test {
    pub name: String,
    pub input: Input,
    pub options: Options,
    pub expected: ast::Circuit,
}

#[derive(Debug, Clone)]
pub enum Input {
    Circuit(ast::Circuit),
    Schematic(ast::Schematic),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Mismatch {
    pub expected: Option<String>,
    pub actual: Option<String>,
}

impl Test {
    /// Compares nodes by name, and the rest of each node by its line, so the order of outputs
    /// counts but the order of nodes doesn't. Positions and pending ticks are not compared.
    pub fn compare(&self, actual: &Graph) -> Vec<Mismatch> {
        let expected = node_lines(&self.expected.graph(&self.options.extract));
        let actual = node_lines(actual);
        let mut matched = vec![false; actual.len()];
        let mut mismatches = Vec::new();
        for (name, expected_line) in expected {
            let found = actual.get_full(&name);
            if let Some((index, _, _)) = found {
                matched[index] = true;
            }
            let actual_line = found.map(|(_, _, line)| line);
            if actual_line != Some(&expected_line) {
                mismatches.push(Mismatch {
                    expected: Some(expected_line),
                    actual: actual_line.cloned(),
                });
            }
        }
        let unmatched = actual
            .into_values()
            .zip(matched)
            .filter(|(_, matched)| !matched);
        mismatches.extend(unmatched.map(|(line, _)| Mismatch {
            expected: None,
            actual: Some(line),
        }));
        mismatches
    }
}

fn node_lines(graph: &Graph) -> IndexMap<String, String> {
    let names = print::NodeNames::new(graph);
    graph
        .node_indices()
        .map(|node| {
            let mut line = String::new();
            print::write_node(&mut line, graph, &names, node)
                .expect("writing to a string succeeds");
            (names.of(node).to_owned(), line)
        })
        .collect()
}
