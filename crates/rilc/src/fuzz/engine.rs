use super::circuit_generator::{random_circuit_case, CircuitCase};
use super::report::{self, caught, illegal_trace, FailedCase, Failure};
use super::rng::Rng;
use super::scenario_generator::{label, moment};
use super::{Cases, GivenCircuits};
use crate::oracle::trace::{Sight, Tracer};
use crate::oracle::{Oracle, Verdict};
use crate::setup::Setup;
use crate::{Error, Outcome};
use anyhow::{bail, Context};
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::graph::NodeState;
use mchprs_redpiler::nodes::{Changes, Nodes};
use mchprs_ril::component;
use mchprs_simulation::readback::Readback;
use mchprs_simulation::Simulation;
use std::path::PathBuf;

const TICKS: u64 = 200;

struct Playing<'a> {
    name: String,
    simulation: Box<dyn Simulation<Nodes>>,
    tracer: Tracer<'a>,
    reported_states: Vec<NodeState>,
}

impl<'a> Playing<'a> {
    fn new(
        setup: &Setup,
        circuit: &'a Circuit,
        observed: &'a [NodeId],
    ) -> Result<Playing<'a>, Error> {
        let mut simulation = setup.build_circuit(circuit)?;
        let sight = Sight::NodesAndPendingTicks;
        Ok(Playing {
            name: setup.to_string(),
            tracer: Tracer::new(&mut *simulation, circuit, observed, sight)?,
            simulation,
            reported_states: circuit.initial().states,
        })
    }

    fn play(&mut self, case: &CircuitCase) -> Result<(), Error> {
        let circuit = &case.circuit;
        for (index, &step) in case.steps.iter().enumerate() {
            let changes = self.tracer.play(&mut *self.simulation, step)?;
            let checked = self.check_changes(circuit, &changes);
            checked.with_context(|| moment(circuit, &case.steps[..=index]))?;
        }
        Ok(())
    }

    fn check_changes(&mut self, circuit: &Circuit, changes: &Changes) -> Result<(), Error> {
        let name = &self.name;
        let state_nodes = changes.states.iter().map(|&(node, _)| node);
        let mut reported_nodes = state_nodes.chain(changes.played_note_blocks.iter().copied());
        if let Some(node) = reported_nodes.find(|&node| !circuit.node(node).observed) {
            let label = label(circuit, node);
            bail!("{name} reports a change of {label}, which is not observed");
        }
        for &(node, state) in &changes.states {
            self.reported_states[node.index()] = state;
        }
        let Readback::Complete(snapshot) = self.simulation.read()? else {
            bail!("{name} reads back incompletely, so its reports can't be checked");
        };
        let states = snapshot.states;
        for node in circuit.ids().filter(|&node| circuit.node(node).observed) {
            let [reported, actual] =
                [&self.reported_states, &states].map(|states| states[node.index()]);
            if reported.shown() != actual.shown() {
                let describe = |state| component(circuit.node(node).kind, state);
                let (label, reported, actual) =
                    (label(circuit, node), describe(reported), describe(actual));
                bail!("{name} last reported {label} {reported}, but it is {label} {actual}");
            }
        }
        Ok(())
    }
}

pub fn command(inputs: &[PathBuf], cases: &Cases) -> Outcome {
    let reference = Setup::reference();
    let direct = Setup::parse("direct")?;
    let given = GivenCircuits::read(inputs)?;
    report::run(cases, |seed| {
        let mut rng = Rng::new(seed);
        let (command, case) = match given.random_case(&mut rng, TICKS) {
            Some((path, case)) => (format!("engine {path}"), case),
            None => {
                let node_counts = rng.pick(&[2..=12, 13..=30, 31..=60]);
                let node_count = rng.within(node_counts);
                let case = random_circuit_case(&mut rng, node_count, TICKS);
                ("engine".to_owned(), case)
            }
        };
        caught(|| check(&case, &direct)).map_err(|problem| {
            let replayed_setups = [&reference, &direct].map(|setup| case.setup(setup));
            Failure {
                command,
                problem,
                case: FailedCase::Circuit(Box::new(case), replayed_setups),
            }
        })
    })
}

fn check(case: &CircuitCase, direct: &Setup) -> Result<Verdict, Error> {
    let circuit = &case.circuit;
    let all_nodes: Vec<NodeId> = circuit.ids().collect();
    let mut playing = Playing::new(direct, circuit, &all_nodes)?;
    playing.play(case)?;
    let verdict = Oracle::new(circuit, &all_nodes).check(&case.steps, playing.tracer.trace());
    verdict
        .map_err(|illegal| illegal_trace(circuit, &all_nodes, &case.steps, &playing.name, illegal))
}
