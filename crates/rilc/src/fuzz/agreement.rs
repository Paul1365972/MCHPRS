use super::circuit_generator::{random_circuit_case, CircuitCase};
use super::report::{self, caught, FailedCase, Failure};
use super::rng::Rng;
use super::scenario_generator::scenario_steps;
use super::{Cases, GivenCircuits};
use crate::diff::{self, CompareEvery};
use crate::numbers::when;
use crate::oracle::Verdict;
use crate::runner::start_circuit;
use crate::scenario::Scenario;
use crate::setup::Setup;
use crate::{Error, Outcome};
use anyhow::bail;
use mchprs_simulation::backend::CompileError;
use std::path::PathBuf;

const TICKS: u64 = 200;

pub fn command(inputs: &[PathBuf], setups: [&Setup; 2], cases: &Cases) -> Outcome {
    let given = GivenCircuits::read(inputs)?;
    let [first, second] = setups;
    let named_setups = format!("-b \"{first}\" -b \"{second}\"");
    report::run(cases, |seed| {
        let mut rng = Rng::new(seed);
        let (command, case) = match given.random_case(&mut rng, TICKS) {
            Some((path, case)) => (format!("agreement {path} {named_setups}"), case),
            None => {
                let node_counts = rng.pick(&[2..=12, 13..=30, 31..=60]);
                let node_count = rng.within(node_counts);
                let case = random_circuit_case(&mut rng, node_count, TICKS);
                (format!("agreement {named_setups}"), case)
            }
        };
        let compare_every = match seed.is_multiple_of(2) {
            true => CompareEvery::Step,
            false => CompareEvery::Tick,
        };
        caught(|| check(&case, setups, seed, compare_every)).map_err(|problem| Failure {
            command,
            problem,
            case: FailedCase::Circuit(Box::new(case), setups.map(Setup::to_string), compare_every),
        })
    })
}

fn check(
    case: &CircuitCase,
    setups: [&Setup; 2],
    seed: u64,
    compare_every: CompareEvery,
) -> Result<Verdict, Error> {
    let [first, second] = setups;
    let start = |setup| match start_circuit(case.graph.clone(), setup) {
        Err(error) if matches!(error.downcast_ref(), Some(CompileError::Unsupported(_))) => {
            Ok(None)
        }
        started => started.map(Some),
    };
    let Some(first_run) = start(first)? else {
        return Ok(Verdict::Skipped);
    };
    let Some(second_run) = start(second)? else {
        return Ok(Verdict::Skipped);
    };
    let mut runs = [first_run, second_run];
    let scenario = Scenario {
        path: PathBuf::from(format!("{seed}.toml")),
        input: PathBuf::from(format!("{seed}.ril")),
        circuit: None,
        settle: None,
        steps: scenario_steps(&case.circuit, &case.steps),
    };
    let played = diff::play(&mut runs, &scenario, setups, compare_every)?;
    if let Some(divergence) = played.divergence {
        bail!(
            "{first} and {second} differ {}\n{divergence}",
            when(played.ticks)
        );
    }
    let [first_run, second_run] = &mut runs;
    if let (Some(first_snapshot), Some(second_snapshot)) =
        (first_run.snapshot()?, second_run.snapshot()?)
        && first_snapshot.kinds == second_snapshot.kinds
    {
        let (mut first_state, mut second_state) = (first_snapshot.state, second_snapshot.state);
        first_state.pending_ticks.sort();
        second_state.pending_ticks.sort();
        if first_state != second_state {
            bail!(
                "{first} and {second} read back different states after {} ticks",
                played.ticks
            );
        }
    }
    match played.components {
        0 => Ok(Verdict::Skipped),
        _ => Ok(Verdict::Passed),
    }
}
