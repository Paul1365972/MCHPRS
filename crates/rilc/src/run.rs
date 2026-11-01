use crate::input::{Input, World};
use crate::numbers::{counted, rate};
use crate::scenario::{Location, Scenario};
use crate::setup::Setup;
use crate::subject::Subject;
use crate::{Error, Outcome};
use mchprs_redpiler::engine::{Drive, Engine};
use mchprs_redpiler::graph::NodeState;
use mchprs_redpiler::ports::Role;
use mchprs_redpiler::simulation::Simulation;
use std::fs;
use std::num::NonZeroUsize;
use std::path::Path;
use std::time::{Duration, Instant};

const SYNC_INTERVAL: Duration = Duration::from_millis(50);

pub fn command(
    scenario_path: &Path,
    setup: &Setup,
    repeats: NonZeroUsize,
    save_outputs: Option<&Path>,
) -> Outcome {
    let scenario = Scenario::read(scenario_path)?;
    let input = scenario.read_input()?;
    let subject = Subject::compile(&input, setup)?;
    let drives = subject.drives(&scenario)?;
    let mut world = match input {
        Input::World(world) => Some(world),
        Input::Circuit(_) => None,
    };
    println!("input: {}", scenario.input.display());
    println!("setup: {setup}");
    println!(
        "compiled in {:.2?}: {}",
        subject.compile_time,
        counted(subject.circuit.len() as u64, "node")
    );
    println!(
        "scenario: {}, {}, synced every {SYNC_INTERVAL:?}",
        counted(scenario.steps.len() as u64, "step"),
        counted(scenario.ticks(), "tick")
    );

    let mut rates = Vec::new();
    let mut sync_times = Vec::new();
    let mut outputs = Vec::new();
    for repeat in 1..=repeats.get() {
        let engine = setup.build(&subject.circuit)?;
        let mut simulation = Simulation::new(subject.circuit.clone(), engine);
        let timing = play(&mut simulation, &scenario, &drives, world.as_mut())?;
        let ticks_per_second = scenario.ticks() as f64 / timing.ticking.as_secs_f64();
        println!(
            "repeat {repeat}: {} ticks/s over {:.2?}, sync {:.2?}",
            rate(ticks_per_second),
            timing.ticking,
            timing.syncing
        );
        rates.push(ticks_per_second);
        sync_times.push(timing.syncing.as_secs_f64());
        if repeat == repeats.get() {
            outputs = output_lines(&subject, &mut simulation)?;
        }
    }
    let [median, min, max] = median_min_max(&mut rates);
    println!(
        "ticks/s: median {}, min {}, max {} over {}",
        rate(median),
        rate(min),
        rate(max),
        counted(repeats.get() as u64, "repeat")
    );
    let [median, min, max] = median_min_max(&mut sync_times).map(Duration::from_secs_f64);
    println!("sync: median {median:.2?}, min {min:.2?}, max {max:.2?}");

    if let Some(path) = save_outputs {
        let mut text = outputs.join("\n");
        text.push('\n');
        fs::write(path, text)
            .map_err(|error| format!("couldn't write {}: {error}", path.display()))?;
    }
    Ok(true)
}

#[derive(Default)]
struct Timing {
    ticking: Duration,
    syncing: Duration,
}

fn play(
    simulation: &mut Simulation,
    scenario: &Scenario,
    drives: &[Option<Drive>],
    mut world: Option<&mut World>,
) -> Result<Timing, Error> {
    let mut timing = Timing::default();
    let mut last_sync = Instant::now();
    let mut batch = 1;
    for (step, drive) in scenario.steps.iter().zip(drives) {
        if let Some(drive) = drive {
            simulation.drive_node(drive.node(), drive.input())?;
        }
        let mut remaining = step.ticks;
        while remaining > 0 {
            let ticks = batch.min(remaining);
            let start = Instant::now();
            simulation.run(ticks)?;
            let elapsed = start.elapsed();
            timing.ticking += elapsed;
            remaining -= ticks;
            let until_sync = SYNC_INTERVAL.saturating_sub(last_sync.elapsed());
            if until_sync.is_zero() {
                let start = Instant::now();
                let changes = simulation.sync()?;
                if let Some(world) = world.as_deref_mut() {
                    changes.apply(world);
                }
                timing.syncing += start.elapsed();
                last_sync = Instant::now();
            } else {
                let rate = ticks as f64 / elapsed.as_secs_f64().max(f64::MIN_POSITIVE);
                let fitting = (rate * until_sync.as_secs_f64()) as u64;
                batch = batch.saturating_mul(2).min(fitting).max(1);
            }
        }
    }
    let start = Instant::now();
    let changes = simulation.sync_all()?;
    if let Some(world) = world {
        changes.apply(world);
    }
    timing.syncing += start.elapsed();
    Ok(timing)
}

fn output_lines<E: Engine>(
    subject: &Subject,
    simulation: &mut Simulation<E>,
) -> Result<Vec<String>, Error> {
    let states = simulation.snapshot()?.states;
    let mut outputs: Vec<(Location, NodeState)> = subject
        .circuit
        .ports()
        .filter(|&(_, role)| role == Role::Output)
        .flat_map(|(id, _)| {
            let state = states[id.index()];
            subject.locations(id).map(move |location| (location, state))
        })
        .collect();
    outputs.sort_by(|(first, _), (second, _)| first.cmp(second));
    Ok(outputs
        .into_iter()
        .map(|(location, state)| {
            let locked = if state.repeater_locked { " locked" } else { "" };
            format!("{location} power {}{locked}", state.power.get())
        })
        .collect())
}

fn median_min_max(values: &mut [f64]) -> [f64; 3] {
    values.sort_by(f64::total_cmp);
    let middle = values.len() / 2;
    let median = if values.len().is_multiple_of(2) {
        (values[middle - 1] + values[middle]) / 2.0
    } else {
        values[middle]
    };
    [median, values[0], values[values.len() - 1]]
}
