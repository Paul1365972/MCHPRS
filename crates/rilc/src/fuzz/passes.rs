use super::circuit_generator::{random_circuit_case, CircuitCase};
use super::report::{self, caught, illegal_trace, FailedCase, Failure};
use super::rng::Rng;
use super::scenario_generator::label;
use super::{Cases, GivenCircuits};
use crate::diff::CompareEvery;
use crate::oracle::reference::ReferenceEngine;
use crate::oracle::trace::{trace, Sight};
use crate::oracle::{Oracle, Step, Verdict};
use crate::setup::Setup;
use crate::{Error, Outcome};
use anyhow::anyhow;
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::passes::{all, Pass};
use mchprs_ril::passes;
use rustc_hash::FxHashMap;
use std::path::PathBuf;

const TICKS: u64 = 60;

pub fn command(inputs: &[PathBuf], list: Option<&str>, cases: &Cases) -> Outcome {
    let given = GivenCircuits::read(inputs)?;
    let pipeline = match list {
        Some(names) => passes::named(names)?,
        None => {
            let setup = Setup::parse("direct --optimize --io-only")?;
            setup.graph_backend()?.passes(setup.flags())
        }
    };
    let alone: Vec<_> = all().into_iter().map(|pass| vec![pass]).collect();
    let lists: Vec<_> = match list {
        Some(_) => vec![pipeline.as_slice()],
        None => alone
            .iter()
            .map(Vec::as_slice)
            .chain((2..=pipeline.len()).map(|len| &pipeline[..len]))
            .collect(),
    };
    report::run(cases, |seed| {
        let mut rng = Rng::new(seed);
        let (command, case) = match given.random_case(&mut rng, TICKS) {
            Some((path, case)) => (format!("passes {path}"), case),
            None => {
                let node_count = rng.within(2..=10);
                let case = random_circuit_case(&mut rng, node_count, TICKS);
                ("passes".to_owned(), case)
            }
        };
        let mut verdict = Verdict::Passed;
        for list in &lists {
            match caught(|| check(&case, list)) {
                Ok(Verdict::Passed) => {}
                Ok(Verdict::Skipped) => verdict = Verdict::Skipped,
                Err(problem) => {
                    let names: Vec<&str> = list.iter().map(|pass| pass.name()).collect();
                    let names = names.join(",");
                    let unoptimized = case.setup(&Setup::reference());
                    let optimized = format!("{unoptimized} --passes={names}");
                    return Err(Failure {
                        command: format!("{command} --passes={names}"),
                        problem,
                        case: FailedCase::Circuit(
                            Box::new(case),
                            [unoptimized, optimized],
                            CompareEvery::Tick,
                        ),
                    });
                }
            }
        }
        Ok(verdict)
    })
}

fn check(case: &CircuitCase, list: &[Box<dyn Pass>]) -> Result<Verdict, Error> {
    let circuit = &case.circuit;
    let mut graph = case.graph.clone();
    for pass in list {
        pass.run(&mut graph);
    }
    let optimized = Circuit::new(graph)
        .map_err(|invalid| anyhow!("the passes leave an invalid circuit: {invalid}"))?;
    let observed: Vec<NodeId> = circuit
        .ids()
        .filter(|&id| circuit.node(id).observed)
        .collect();
    let showing = nodes_showing(circuit, &observed, &optimized)?;
    let mut optimized_steps = Vec::new();
    for &step in &case.steps {
        optimized_steps.push(match step {
            Step::Tick => Step::Tick,
            Step::Drive(node, input) => {
                let observed_index = observed.binary_search(&node);
                let showing_node = showing[observed_index.expect("inputs are observed")];
                let drive = optimized.input_kinds().accept(showing_node, input);
                let label = label(&optimized, showing_node);
                drive.map_err(|refusal| anyhow!("after the passes, {label} {refusal}"))?;
                Step::Drive(showing_node, input)
            }
        });
    }
    let engine = &mut ReferenceEngine::new(&optimized);
    let trace = trace(engine, &optimized, &showing, Sight::Nodes, &optimized_steps)?;
    let verdict = Oracle::new(circuit, &observed).check(&case.steps, &trace);
    let seen_by = "the circuit after the passes";
    verdict.map_err(|illegal| illegal_trace(circuit, &observed, &case.steps, seen_by, illegal))
}

fn nodes_showing(
    circuit: &Circuit,
    observed: &[NodeId],
    optimized: &Circuit,
) -> Result<Vec<NodeId>, Error> {
    let mut observed_by_name = FxHashMap::default();
    for id in optimized.ids().filter(|&id| optimized.node(id).observed) {
        for name in &optimized.node(id).names {
            observed_by_name.insert(name.as_str(), id);
        }
    }
    let mut showing = Vec::new();
    for &id in observed {
        let name = circuit.node(id).names[0].as_str();
        let showing_node = observed_by_name.get(name);
        let unobserved = || anyhow!("%{name} is not observed after the passes");
        showing.push(*showing_node.ok_or_else(unobserved)?);
    }
    Ok(showing)
}
