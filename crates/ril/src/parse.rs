mod component;
mod token;

use crate::ast::{Circuit, Output, Schematic};
use crate::extraction;
use crate::passes::{self, UnknownPass, PASSES};
use crate::{Error, Input, Module, Options, Test};
use component::Tick;
use mchprs_redpiler::circuit::{self, InvalidCircuit};
use mchprs_redpiler::ports::WireOutputs;
use mchprs_redstone::tick_queue::PendingTick;
use mchprs_simulation::backend::STALE;
use mchprs_syntax::flag::{Flag, FlagValues, InvalidFlags};
use mchprs_syntax::reader::{Reader, SyntaxError};
use rustc_hash::{FxHashMap, FxHashSet};

type Parsed<T> = Result<T, SyntaxError>;

struct TestArgs {
    options: Options,
    world_only_flag: Option<Flag>,
}

#[derive(Clone, Copy)]
enum Item {
    Circuit,
    Schematic,
    Test,
    TestArgs,
}

const ITEMS: [(&str, Item); 4] = [
    ("circuit", Item::Circuit),
    ("schematic", Item::Schematic),
    ("test", Item::Test),
    ("test_args", Item::TestArgs),
];

enum Global {
    Circuit(usize),
    Schematic(usize),
    Test,
}

#[derive(Debug, thiserror::Error)]
enum OptionsError {
    #[error(transparent)]
    Flags(#[from] InvalidFlags),
    #[error("missing --{}", PASSES.name())]
    MissingPasses,
    #[error(transparent)]
    UnknownPass(#[from] UnknownPass),
}

pub fn parse(source: &str) -> Result<Module, Error> {
    let mut parser = Parser {
        reader: Reader::new(source),
        globals: FxHashMap::default(),
        test_args: None,
        circuits: Vec::new(),
        schematics: Vec::new(),
        tests: Vec::new(),
    };
    match parser.module() {
        Ok(()) => Ok(Module {
            source: source.to_owned(),
            circuits: parser.circuits,
            tests: parser.tests,
        }),
        Err(error) => Err(Error::at(source, error.cursor, error.message)),
    }
}

struct Parser<'a> {
    reader: Reader<'a>,
    globals: FxHashMap<String, Global>,
    test_args: Option<String>,
    circuits: Vec<Circuit>,
    schematics: Vec<Schematic>,
    tests: Vec<Test>,
}

impl Parser<'_> {
    fn module(&mut self) -> Parsed<()> {
        loop {
            self.skip();
            if !self.reader.can_read() {
                return Ok(());
            }
            let start = self.reader.cursor();
            match self.keyword(&ITEMS, "circuit, schematic, test or test_args")? {
                Item::Circuit => {
                    let circuit = self.circuit(start)?;
                    let global = Global::Circuit(self.circuits.len());
                    self.globals.insert(circuit.name.clone(), global);
                    self.circuits.push(circuit);
                }
                Item::Schematic => {
                    let name = self.global_name()?;
                    let path = self.string()?;
                    let global = Global::Schematic(self.schematics.len());
                    self.globals.insert(name.clone(), global);
                    self.schematics.push(Schematic { name, path });
                }
                Item::Test => self.test(start)?,
                Item::TestArgs => {
                    if self.test_args.is_some() {
                        return Err(self.fail(start, "duplicate test_args"));
                    }
                    self.skip();
                    let at = self.reader.cursor();
                    let arguments = self.string()?;
                    parse_test_args(&arguments)
                        .map_err(|error| self.fail(at, error.to_string()))?;
                    self.test_args = Some(arguments);
                }
            }
        }
    }

    fn test(&mut self, start: usize) -> Parsed<()> {
        self.require('(')?;
        self.require('@')?;
        let input_at = self.reader.cursor();
        let input_name = self.name("a circuit or schematic")?;
        let input = match self.globals.get(input_name) {
            Some(&Global::Circuit(index)) => Input::Circuit(self.circuits[index].clone()),
            Some(&Global::Schematic(index)) => Input::Schematic(self.schematics[index].clone()),
            Some(Global::Test) => {
                let message = format!("@{input_name} is a test, not a circuit or schematic");
                return Err(self.fail(input_at, message));
            }
            None => {
                let message = format!("unknown circuit or schematic @{input_name}");
                return Err(self.fail(input_at, message));
            }
        };
        let TestArgs {
            options,
            world_only_flag,
        } = if self.eat(',') {
            let test_args = self.test_args()?;
            self.require(')')?;
            test_args
        } else {
            self.require(')')?;
            match &self.test_args {
                Some(arguments) => parse_test_args(arguments)
                    .map_err(|error| self.fail(start, error.to_string()))?,
                None => return Err(self.fail(start, "missing test_args")),
            }
        };
        if let (Input::Circuit(_), Some(flag)) = (&input, world_only_flag) {
            let message = format!("--{} only applies to schematic inputs", flag.name());
            return Err(self.fail(input_at, message));
        }
        self.skip();
        let circuit_at = self.reader.cursor();
        if !self.eat_word("circuit") {
            return Err(self.fail(circuit_at, "expected circuit"));
        }
        let expected = self.circuit(circuit_at)?;
        self.globals.insert(expected.name.clone(), Global::Test);
        self.tests.push(Test {
            name: expected.name.clone(),
            input,
            options,
            expected,
        });
        Ok(())
    }

    fn circuit(&mut self, start: usize) -> Parsed<Circuit> {
        let name = self.global_name()?;
        self.require('{')?;
        let mut components = Vec::new();
        let mut starts = Vec::new();
        let mut ticks = Vec::new();
        let mut named_outputs = Vec::new();
        let mut indices = FxHashMap::default();
        let end = loop {
            if self.eat('}') {
                break self.reader.cursor();
            }
            starts.push(self.reader.cursor());
            let (component, tick, outputs) = self.component(&mut indices, components.len())?;
            components.push(component);
            ticks.push(tick);
            named_outputs.push(outputs);
        };

        for (component, outputs) in components.iter_mut().zip(named_outputs) {
            let mut linked = FxHashSet::default();
            for output in outputs {
                let Some(&target) = indices.get(&output.target) else {
                    let message = format!("unknown value %{}", output.target);
                    return Err(self.fail(output.at, message));
                };
                if !linked.insert((target, output.link.kind)) {
                    let message = format!("a second link of this kind to %{}", output.target);
                    return Err(self.fail(output.at, message));
                }
                component.outputs.push(Output {
                    target,
                    link: output.link,
                });
            }
        }
        self.check_orders(&ticks)?;

        let circuit = Circuit {
            name,
            components,
            span: start..end,
        };
        if let Err(invalid) = circuit::Circuit::check(&circuit.graph()) {
            return Err(match invalid {
                InvalidCircuit::Node { node, problem } => {
                    let label = &circuit.components[node.index()].labels[0];
                    self.fail(starts[node.index()], format!("%{label} {problem}"))
                }
                InvalidCircuit::Stale { node } => {
                    let label = &circuit.components[node.index()].labels[0];
                    self.fail(starts[node.index()], format!("%{label} {STALE}"))
                }
                InvalidCircuit::TooManyNodes(count) => {
                    let name = &circuit.name;
                    let message = format!("invalid circuit @{name}: {count} nodes exceed u32");
                    self.fail(start, message)
                }
            });
        }
        Ok(circuit)
    }

    fn check_orders(&mut self, ticks: &[Option<Tick>]) -> Parsed<()> {
        let mut due_together: FxHashMap<PendingTick, Vec<Option<usize>>> = FxHashMap::default();
        for tick in ticks.iter().flatten() {
            due_together.entry(tick.tick).or_default().push(tick.order);
        }
        for tick in ticks.iter().flatten() {
            let orders = &due_together[&tick.tick];
            let message = match tick.order {
                None if orders.len() > 1 => {
                    "ticks due together with the same priority need an order".to_owned()
                }
                Some(_) if orders.len() == 1 => {
                    "an order needs another tick due together with the same priority".to_owned()
                }
                Some(order) if orders.iter().filter(|&&other| other == Some(order)).count() > 1 => {
                    format!("duplicate order {order}")
                }
                _ => continue,
            };
            return Err(self.fail(tick.at, message));
        }
        Ok(())
    }

    fn test_args(&mut self) -> Parsed<TestArgs> {
        self.skip();
        let at = self.reader.cursor();
        let arguments = self.string()?;
        parse_test_args(&arguments).map_err(|error| self.fail(at, error.to_string()))
    }
}

fn parse_test_args(arguments: &str) -> Result<TestArgs, OptionsError> {
    let accepted = [[PASSES].as_slice(), &extraction::FLAGS].concat();
    let flags = FlagValues::parse(arguments, &accepted)?;
    let names = flags.text(&PASSES).ok_or(OptionsError::MissingPasses)?;
    let options = Options {
        extract: extraction::options(&flags),
        passes: passes::named(names)?,
    };
    let world_only_flag = WireOutputs::FLAGS
        .into_iter()
        .find(|flag| flags.is_on(flag));
    Ok(TestArgs {
        options,
        world_only_flag,
    })
}
