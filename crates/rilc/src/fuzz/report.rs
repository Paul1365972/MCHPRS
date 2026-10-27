use super::circuit_generator::CircuitCase;
use super::scenario_generator::{label, moment, scenario_toml};
use super::Cases;
use crate::diff::CompareEvery;
use crate::numbers::{counted, grouped};
use crate::oracle::trace::{NodeView, PendingTicks, View};
use crate::oracle::{Illegal, Step, Verdict};
use crate::{Error, Outcome};
use anyhow::{anyhow, Context};
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_ril::{component, Printed};
use std::fs;
use std::panic::{self, AssertUnwindSafe};
use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};
use std::thread;

const FAILED_CASE_DIRECTORY: &str = ".scratch/fuzz";
const MOST_LEGAL_VIEWS_DESCRIBED: usize = 4;

pub struct Failure {
    pub command: String,
    pub problem: Error,
    pub case: FailedCase,
}

impl Failure {
    fn print(&self, seed: u64) -> Result<(), Error> {
        println!("FAILED rilc fuzz {} --seed {seed} --cases 1", self.command);
        println!("{:#}", self.problem);
        match &self.case {
            FailedCase::World(described) => println!("{described}"),
            FailedCase::Circuit(case, [first, second], compare_every) => {
                let write = |extension: &str, text: &str| {
                    let path = Path::new(FAILED_CASE_DIRECTORY).join(format!("{seed}.{extension}"));
                    fs::write(&path, text)
                        .with_context(|| format!("couldn't write {}", path.display()))?;
                    Ok::<_, Error>(path)
                };
                fs::create_dir_all(FAILED_CASE_DIRECTORY)
                    .with_context(|| format!("couldn't create {FAILED_CASE_DIRECTORY}"))?;
                let ril = Printed::new(&case.graph, &format!("seed_{seed}")).to_string();
                let toml = scenario_toml(&case.circuit, &case.steps, seed)?;
                write("ril", &ril)?;
                let scenario_path = write("toml", &toml)?;
                println!("{ril}\n{toml}");
                let replay = scenario_path.display();
                let steps = match compare_every {
                    CompareEvery::Tick => "",
                    CompareEvery::Step => " --steps",
                };
                println!("rilc diff {replay}{steps} -b \"{first}\" -b \"{second}\"");
            }
        }
        println!();
        Ok(())
    }
}

pub enum FailedCase {
    Circuit(Box<CircuitCase>, [String; 2], CompareEvery),
    World(String),
}

struct Checking {
    seed: u64,
}

impl Drop for Checking {
    fn drop(&mut self) {
        if thread::panicking() {
            let seed = self.seed;
            eprintln!("the case of seed {seed} panicked; replay it with --seed {seed} --cases 1");
        }
    }
}

pub fn run(cases: &Cases, check: impl Fn(u64) -> Result<Verdict, Failure> + Sync) -> Outcome {
    let next_index = AtomicU64::new(0);
    let check_cases = || {
        let mut checked = Vec::new();
        loop {
            let index = next_index.fetch_add(1, Ordering::Relaxed);
            if index >= cases.count {
                return checked;
            }
            let seed = cases.seed.wrapping_add(index);
            let _checking = Checking { seed };
            checked.push((index, check(seed)));
        }
    };
    let threads = match cases.jobs {
        Some(jobs) => jobs.get(),
        None => thread::available_parallelism().map_or(1, usize::from),
    };
    let mut checked: Vec<_> = thread::scope(|scope| {
        let workers: Vec<_> = (0..threads).map(|_| scope.spawn(check_cases)).collect();
        let checked = workers.into_iter().flat_map(|worker| {
            let checked = worker.join();
            checked.expect("no case panics outside its caught check")
        });
        checked.collect()
    });
    checked.sort_by_key(|&(index, _)| index);
    let (mut passed, mut failed, mut skipped) = (0, 0, 0);
    for (index, verdict) in checked {
        match verdict {
            Ok(Verdict::Passed) => passed += 1,
            Ok(Verdict::Skipped) => skipped += 1,
            Err(failure) => {
                failed += 1;
                failure.print(cases.seed.wrapping_add(index))?;
            }
        }
    }
    println!(
        "{}, {} passed, {} failed, {} skipped",
        counted(cases.count, "case"),
        grouped(passed),
        grouped(failed),
        grouped(skipped)
    );
    Ok(failed == 0 && passed > 0)
}

pub fn illegal_trace(
    circuit: &Circuit,
    observed: &[NodeId],
    steps: &[Step],
    seen_by: &str,
    illegal: Illegal,
) -> Error {
    let Illegal {
        steps_played,
        seen,
        legal,
    } = illegal;
    let differing: Vec<usize> = (0..observed.len())
        .filter(|&index| {
            legal
                .iter()
                .any(|view| view.nodes[index] != seen.nodes[index])
        })
        .collect();
    let pending_ticks_differ = legal
        .iter()
        .any(|view| view.pending_ticks != seen.pending_ticks);
    let describe = |view: &View| {
        let mut described = Vec::new();
        for &index in &differing {
            let id = observed[index];
            let NodeView { state, played_note } = view.nodes[index];
            let component = component(circuit.node(id).kind, state);
            let played = if played_note { " played" } else { "" };
            described.push(format!("{} {component}{played}", label(circuit, id)));
        }
        if pending_ticks_differ {
            let ticks = view.pending_ticks.iter().flat_map(PendingTicks::iter);
            let ticks: Vec<String> = ticks
                .map(|&(id, tick)| format!("{} in {tick}", label(circuit, id)))
                .collect();
            described.push(format!("pending ticks [{}]", ticks.join(", ")));
        }
        described.join(", ")
    };
    let mut lines = vec![format!("{seen_by}: {}", describe(&seen))];
    for view in legal.iter().take(MOST_LEGAL_VIEWS_DESCRIBED) {
        lines.push(format!("legal: {}", describe(view)));
    }
    if legal.len() > MOST_LEGAL_VIEWS_DESCRIBED {
        let more = legal.len() - MOST_LEGAL_VIEWS_DESCRIBED;
        lines.push(format!("and {more} more legal views"));
    }
    anyhow!(
        "no order the circuit may run in shows what {seen_by} shows {}\n{}",
        moment(circuit, &steps[..steps_played]),
        lines.join("\n")
    )
}

pub fn caught<T>(check: impl FnOnce() -> Result<T, Error>) -> Result<T, Error> {
    panic::catch_unwind(AssertUnwindSafe(check)).unwrap_or_else(|panic| {
        let message = match (panic.downcast_ref::<&str>(), panic.downcast_ref::<String>()) {
            (Some(message), _) => message,
            (None, Some(message)) => message.as_str(),
            (None, None) => "no message",
        };
        Err(anyhow!("panicked: {message}"))
    })
}
