use crate::input::{Loaded, World};
use crate::numbers::{counted, rate};
use crate::runner::labelled_graph;
use crate::scenario::{Scenario, Target};
use crate::setup::Setup;
use crate::{Error, Outcome};
use anyhow::{bail, Context};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::BlockPos;
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::nodes::Changes;
use mchprs_ril::ast;
use mchprs_simulation::component::strength_at;
use mchprs_simulation::layout::Layout;
use mchprs_simulation::ports::Role;
use mchprs_simulation::{Command, Event, Run, Running, Space, Update, View};
use std::fs;
use std::num::NonZeroUsize;
use std::path::Path;
use std::thread;
use std::time::{Duration, Instant};

pub fn command(
    scenario_path: &Path,
    setup: &Setup,
    repeats: NonZeroUsize,
    unlimited: Option<Duration>,
    save_outputs: Option<&Path>,
) -> Outcome {
    let scenario = Scenario::read(scenario_path)?;
    let input = scenario.read_input()?;
    println!("input: {}", scenario.input.display());
    println!("setup: {setup}");
    println!(
        "scenario: {}, {}",
        counted(scenario.steps.len() as u64, "step"),
        counted(scenario.ticks(), "tick")
    );

    let mut rates = Vec::new();
    let mut unlimited_rates = Vec::new();
    for repeat in 1..=repeats.get() {
        let shows_outputs = repeat == repeats.get() && save_outputs.is_some();
        let start_time = Instant::now();
        let (timed, outputs) = match &input {
            Loaded::World(world) => time_world(world, setup, &scenario, unlimited, shows_outputs)?,
            Loaded::Circuit(circuit) => {
                time_circuit(circuit, setup, &scenario, unlimited, shows_outputs)?
            }
        };
        let setup_time = timed.started - start_time;
        let ticks_per_second = scenario.ticks() as f64 / timed.ticking.as_secs_f64();
        let mut line = format!(
            "repeat {repeat}: set up in {setup_time:.2?}, {} ticks/s over {:.2?}",
            rate(ticks_per_second),
            timed.ticking,
        );
        rates.push(ticks_per_second);
        if let Some((ticks, elapsed)) = timed.unlimited {
            let ticks_per_second = ticks as f64 / elapsed.as_secs_f64();
            line.push_str(&format!(
                ", {} ticks/s unlimited over {elapsed:.2?}",
                rate(ticks_per_second)
            ));
            unlimited_rates.push(ticks_per_second);
        }
        println!("{line}");
        if let (true, Some(path)) = (shows_outputs, save_outputs) {
            let mut text = outputs.join("\n");
            text.push('\n');
            fs::write(path, text).with_context(|| format!("couldn't write {}", path.display()))?;
        }
    }
    print_summary("ticks/s", &mut rates);
    if unlimited.is_some() {
        print_summary("unlimited ticks/s", &mut unlimited_rates);
    }
    Ok(true)
}

pub fn seconds(text: &str) -> Result<Duration, String> {
    let seconds: f64 = text.parse().map_err(|error| format!("{error}"))?;
    Duration::try_from_secs_f64(seconds).map_err(|error| error.to_string())
}

fn time_world(
    world: &World,
    setup: &Setup,
    scenario: &Scenario,
    unlimited: Option<Duration>,
    shows_outputs: bool,
) -> Result<(Timed, Vec<String>), Error> {
    let (simulation, layout) = setup.compile(world)?;
    let running = simulation.start()?;
    let inputs = world_inputs(&layout, scenario)?;
    let (timed, changes) = time(running, &inputs, scenario, unlimited, shows_outputs)?;
    let mut blocks = world.blocks().clone();
    changes.write(&mut blocks);
    Ok((timed, world_outputs(&blocks, &layout)))
}

fn time_circuit(
    circuit: &ast::Circuit,
    setup: &Setup,
    scenario: &Scenario,
    unlimited: Option<Duration>,
    shows_outputs: bool,
) -> Result<(Timed, Vec<String>), Error> {
    let (circuit, simulation) = setup.build(labelled_graph(circuit))?;
    let running = simulation.start()?;
    let inputs = circuit_inputs(&circuit, scenario)?;
    let (timed, changes) = time(running, &inputs, scenario, unlimited, shows_outputs)?;
    Ok((timed, circuit_outputs(&circuit, changes)))
}

struct Timed {
    started: Instant,
    ticking: Duration,
    unlimited: Option<(u64, Duration)>,
}

/// Returns what was shown, ending with everything when `shows_outputs`.
fn time<S: Space>(
    mut running: Box<dyn Running<S>>,
    inputs: &[Option<S::Address>],
    scenario: &Scenario,
    unlimited: Option<Duration>,
    shows_outputs: bool,
) -> Result<(Timed, S::Changes), Error> {
    let started = Instant::now();
    let mut shown = S::Changes::default();
    let mut ticking = Duration::ZERO;
    for (step, input) in scenario.steps.iter().zip(inputs) {
        if let (Some(action), Some(at)) = (&step.action, input) {
            running.send(Command::Drive(*at, action.input));
        }
        let start = Instant::now();
        running.send(Command::Run(Run::Ticks(step.ticks)));
        wait_for(&mut *running, Event::RunFinished, &mut shown)?;
        ticking += start.elapsed();
    }
    let unlimited = match unlimited {
        Some(duration) => {
            let start = Instant::now();
            running.send(Command::Run(Run::Unlimited));
            thread::sleep(duration);
            running.send(Command::Show(View::Observed));
            let update = wait_for(&mut *running, Event::Shown(View::Observed), &mut shown)?;
            let elapsed = start.elapsed();
            running.send(Command::Pause);
            Some((update.ticks.saturating_sub(scenario.ticks()), elapsed))
        }
        None => None,
    };
    if shows_outputs {
        running.send(Command::Show(View::All));
        wait_for(&mut *running, Event::Shown(View::All), &mut shown)?;
    }
    let timed = Timed {
        started,
        ticking,
        unlimited,
    };
    Ok((timed, shown))
}

fn wait_for<S: Space>(
    running: &mut dyn Running<S>,
    wanted: Event<S::Address>,
    shown: &mut S::Changes,
) -> Result<Update<S>, Error> {
    loop {
        let mut update = running.wait()?;
        shown.extend([std::mem::take(&mut update.changes)]);
        for event in &update.events {
            match event {
                Event::Refused(at, error) => bail!("the input at {at:?} {error}"),
                Event::Halted(_) | Event::RunInterrupted => {
                    bail!("the run ended before its goal")
                }
                Event::RunFinished | Event::Shown(_) => {}
            }
        }
        if update.events.contains(&wanted) {
            return Ok(update);
        }
    }
}

fn world_inputs(layout: &Layout, scenario: &Scenario) -> Result<Vec<Option<BlockPos>>, Error> {
    let inputs = scenario.steps.iter().enumerate().map(|(index, step)| {
        let Some(action) = &step.action else {
            return Ok(None);
        };
        let Target::Position([x, y, z]) = action.at else {
            return Err(scenario.step_error(index, "a world's inputs are positions"));
        };
        let pos = BlockPos::new(x, y, z);
        let port = layout.at(pos).and_then(|component| component.port);
        if port != Some(Role::Input) {
            let problem = format!("{} is not an input of the setup", action.at);
            return Err(scenario.step_error(index, problem));
        }
        Ok(Some(pos))
    });
    inputs.collect()
}

fn circuit_inputs(circuit: &Circuit, scenario: &Scenario) -> Result<Vec<Option<NodeId>>, Error> {
    let inputs = scenario.steps.iter().enumerate().map(|(index, step)| {
        let Some(action) = &step.action else {
            return Ok(None);
        };
        let Target::Name(label) = &action.at else {
            return Err(scenario.step_error(index, "a circuit's inputs are labels like %name"));
        };
        let mut inputs = circuit.ids().filter(|&id| circuit.node(id).is_input());
        let node = inputs.find(|&id| circuit.node(id).names.contains(label));
        let problem = || format!("{} is not an input of the setup", action.at);
        node.map(Some)
            .ok_or_else(|| scenario.step_error(index, problem()))
    });
    inputs.collect()
}

fn world_outputs(blocks: &impl BlockView, layout: &Layout) -> Vec<String> {
    let mut outputs: Vec<BlockPos> = layout
        .positions()
        .filter(|&pos| layout.at(pos).and_then(|component| component.port) == Some(Role::Output))
        .collect();
    outputs.sort();
    let mut lines = Vec::new();
    for pos in outputs {
        let Some(power) = strength_at(blocks, pos) else {
            continue;
        };
        let block = blocks.block(pos);
        let locked = matches!(block, Block::Repeater(repeater) if repeater.locked);
        let locked = if locked { " locked" } else { "" };
        let target = Target::from(pos);
        lines.push(format!("{target} power {}{locked}", power.get()));
    }
    lines
}

fn circuit_outputs(circuit: &Circuit, shown: Changes) -> Vec<String> {
    let mut states = circuit.initial().states;
    for (node, state) in shown.states {
        states[node.index()] = state;
    }
    let mut lines: Vec<String> = circuit
        .ids()
        .filter(|&id| circuit.node(id).port == Some(Role::Output))
        .flat_map(|id| {
            let state = states[id.index()];
            let locked = if state.repeater_locked { " locked" } else { "" };
            let names = circuit.node(id).names.iter();
            names.map(move |name| {
                let target = Target::Name(name.clone());
                format!("{target} power {}{locked}", state.power.get())
            })
        })
        .collect();
    lines.sort();
    lines
}

fn print_summary(label: &str, rates: &mut [f64]) {
    let [median, min, max] = median_min_max(rates);
    println!(
        "{label}: median {}, min {}, max {} over {}",
        rate(median),
        rate(min),
        rate(max),
        counted(rates.len() as u64, "repeat")
    );
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

#[cfg(test)]
mod tests {
    use super::time_circuit;
    use crate::scenario::{Action, Scenario, Step, Target};
    use crate::setup::Setup;
    use mchprs_redstone::input::Input;
    use mchprs_ril::Module;

    #[test]
    fn a_circuit_runs_by_itself_and_is_driven_by_label() {
        let module = Module::parse("circuit @c { %lever = lever -> [%lamp:0] %lamp = lamp }");
        let module = module.expect("the circuit parses");
        let toggle = Action {
            at: Target::Name("lever".to_owned()),
            input: Input::Toggle,
        };
        let scenario = Scenario {
            path: "lamp.toml".into(),
            input: "lamp.ril".into(),
            circuit: None,
            settle: None,
            steps: vec![Step {
                action: Some(toggle),
                ticks: 2,
            }],
        };
        let setup = Setup::parse("direct").expect("direct is a setup");
        let timed = time_circuit(&module.circuits()[0], &setup, &scenario, None, true);
        let (_, outputs) = timed.expect("the circuit runs");
        assert_eq!(outputs, ["%lamp power 15"]);
    }
}
