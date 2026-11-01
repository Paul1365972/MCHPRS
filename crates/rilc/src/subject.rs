use crate::compile::compile;
use crate::input::Input;
use crate::scenario::{Location, Scenario};
use crate::setup::Setup;
use crate::Error;
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::engine::{Drive, DriveError};
use rustc_hash::FxHashMap;
use std::time::{Duration, Instant};

pub struct Subject {
    pub circuit: Circuit,
    pub compile_time: Duration,
    is_circuit: bool,
    nodes: FxHashMap<Location, NodeId>,
}

impl Subject {
    pub fn compile(input: &Input, setup: &Setup) -> Result<Subject, Error> {
        let start = Instant::now();
        let (graph, _) = compile(input, setup, &mut |_, _| {})?;
        let circuit = Circuit::new(graph).expect("compiles keep the circuit invariants");
        let compile_time = start.elapsed();
        let mut subject = Subject {
            circuit,
            compile_time,
            is_circuit: matches!(input, Input::Circuit(_)),
            nodes: FxHashMap::default(),
        };
        let nodes = subject
            .circuit
            .ids()
            .flat_map(|id| subject.locations(id).map(move |location| (location, id)))
            .collect();
        subject.nodes = nodes;
        Ok(subject)
    }

    pub fn is_circuit(&self) -> bool {
        self.is_circuit
    }

    pub fn node_at(&self, location: &Location) -> Option<NodeId> {
        self.nodes.get(location).copied()
    }

    pub fn nodes(&self) -> impl Iterator<Item = (&Location, NodeId)> {
        self.nodes.iter().map(|(location, &id)| (location, id))
    }

    pub fn locations(&self, id: NodeId) -> impl Iterator<Item = Location> + '_ {
        let node = self.circuit.node(id);
        let names = node.names.iter().cloned().map(Location::Name);
        names.chain(node.blocks.iter().map(|&(pos, _)| Location::Position(pos)))
    }

    pub fn drives(&self, scenario: &Scenario) -> Result<Vec<Option<Drive>>, Error> {
        let mut drives = Vec::with_capacity(scenario.steps.len());
        for (index, step) in scenario.steps.iter().enumerate() {
            let Some(action) = &step.action else {
                drives.push(None);
                continue;
            };
            let Some(node) = self.node_at(&action.location) else {
                let problem = format!("no node at {}", action.location);
                return Err(scenario.step_error(index, problem));
            };
            let drive = self
                .circuit
                .check_input(node, action.input)
                .map_err(|error| {
                    let component = self.describe(node);
                    let problem = match error {
                        DriveError::NotAnInput(_) => {
                            format!("{} is {component}, which is not an input", action.location)
                        }
                        _ => format!("can't {action}, which is {component}"),
                    };
                    scenario.step_error(index, problem)
                })?;
            drives.push(Some(drive));
        }
        Ok(drives)
    }

    pub fn describe(&self, node: NodeId) -> String {
        let node = self.circuit.node(node);
        mchprs_ril::component(&node.kind, node.state).to_string()
    }
}
