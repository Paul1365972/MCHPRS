pub mod ast;
pub mod extraction;
mod fields;
mod parse;
pub mod passes;
mod print;
mod string_replacer;

use indexmap::IndexMap;
use mchprs_redpiler::extract::ExtractOptions;
use mchprs_redpiler::graph::Graph;
use mchprs_redpiler::passes::Pass;
use print::TickOrders;
use string_replacer::StringReplacer;

pub use print::{component, NodeNames, Printed};

/// `line` and `column` count from 1.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("{line}:{column}: {message}")]
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
}

#[derive(Debug)]
pub struct Module {
    source: String,
    circuits: Vec<ast::Circuit>,
    tests: Vec<Test>,
}

impl Module {
    pub fn parse(source: &str) -> Result<Module, Error> {
        parse::parse(source)
    }

    pub fn circuits(&self) -> &[ast::Circuit] {
        &self.circuits
    }

    pub fn circuit(&self, name: &str) -> Option<&ast::Circuit> {
        self.circuits.iter().find(|circuit| circuit.name == name)
    }

    pub fn tests(&self) -> &[Test] {
        &self.tests
    }

    pub fn with_results(&self, results: &[(&Test, Graph)]) -> String {
        let mut replacements: Vec<_> = results
            .iter()
            .map(|(test, graph)| {
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

#[derive(Debug)]
pub struct Test {
    pub name: String,
    pub input: Input,
    pub options: Options,
    pub expected: ast::Circuit,
}

impl Test {
    pub fn compare(&self, actual: &Graph) -> Vec<Mismatch> {
        let expected = node_lines(&self.expected.graph());
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

#[derive(Debug, Clone)]
pub enum Input {
    Circuit(ast::Circuit),
    Schematic(ast::Schematic),
}

#[derive(Debug)]
pub struct Options {
    pub extract: ExtractOptions,
    pub passes: Vec<Box<dyn Pass>>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Mismatch {
    pub expected: Option<String>,
    pub actual: Option<String>,
}

fn node_lines(graph: &Graph) -> IndexMap<String, String> {
    let names = NodeNames::new(graph);
    let orders = TickOrders::new(graph);
    graph
        .node_indices()
        .map(|node| {
            let mut line = String::new();
            print::write_node(&mut line, graph, &names, &orders, node, false)
                .expect("writing to a string succeeds");
            (names.of(node).to_owned(), line)
        })
        .collect()
}
