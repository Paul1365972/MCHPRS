use super::rng::Rng;
use crate::numbers::{counted, when};
use crate::oracle::Step;
use crate::scenario::{self, Action, Target};
use crate::Error;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::graph::DriveKind;
use mchprs_redstone::input::Input;
use std::mem;
use std::path::Path;

pub const WARM_UP_TICKS: u64 = 40;
const LONGEST_STRETCH_TICKS: u64 = 20;
const MOST_INPUTS_BEFORE_TICK: u64 = 3;

pub fn random_steps(circuit: &Circuit, rng: &mut Rng, ticks: u64) -> Vec<Step> {
    let inputs: Vec<(NodeId, DriveKind)> = circuit
        .ids()
        .filter(|&id| circuit.node(id).is_input())
        .filter_map(|id| Some((id, circuit.node(id).kind.traits().drive?)))
        .collect();
    let mut steps = Vec::new();
    let mut remaining_ticks = ticks;
    while remaining_ticks > 0 {
        let is_quiet = inputs.is_empty() || rng.chance(50);
        let stretch_ticks = rng.within(1..=LONGEST_STRETCH_TICKS).min(remaining_ticks);
        remaining_ticks -= stretch_ticks;
        for _ in 0..stretch_ticks {
            if !is_quiet {
                for _ in 0..rng.within(1..=MOST_INPUTS_BEFORE_TICK) {
                    let (node, kind) = rng.pick(&inputs);
                    steps.push(Step::Drive(node, random_input(kind, rng)));
                }
            }
            steps.push(Step::Tick);
        }
    }
    steps
}

fn random_input(kind: DriveKind, rng: &mut Rng) -> Input {
    match kind {
        DriveKind::OnOff => rng.pick(&[
            Input::Toggle,
            Input::Set(SignalStrength::Zero),
            Input::Set(SignalStrength::Fifteen),
        ]),
        DriveKind::Press => Input::Press,
        DriveKind::Touch => rng.pick(&[Input::Toggle, Input::Touch(false), Input::Touch(true)]),
        DriveKind::Strength => Input::Set(random_strength(rng)),
    }
}

pub fn random_strength(rng: &mut Rng) -> SignalStrength {
    SignalStrength::try_from(rng.below(16) as u8).expect("below 16 is a signal strength")
}

pub fn moment(circuit: &Circuit, played: &[Step]) -> String {
    let (inputs, ticks) = timed_inputs(circuit, played);
    let last_input = inputs.last().map_or("no input", String::as_str);
    match played.last() {
        Some(Step::Drive(..)) => format!("after {last_input}"),
        Some(Step::Tick) | None => format!("{}, the last input being {last_input}", when(ticks)),
    }
}

pub fn describe(circuit: &Circuit, steps: &[Step]) -> String {
    let (mut lines, ticks) = timed_inputs(circuit, steps);
    lines.push(counted(ticks, "tick"));
    lines.join("\n")
}

fn timed_inputs(circuit: &Circuit, steps: &[Step]) -> (Vec<String>, u64) {
    let mut inputs = Vec::new();
    let mut ticks = 0;
    for &step in steps {
        match step {
            Step::Tick => ticks += 1,
            Step::Drive(node, input) => {
                let action = action(circuit, node, input);
                inputs.push(format!("{action} before tick {}", ticks + 1));
            }
        }
    }
    (inputs, ticks)
}

pub fn scenario_toml(circuit: &Circuit, steps: &[Step], seed: u64) -> Result<String, Error> {
    let mut written = Vec::new();
    let mut current = scenario::Step {
        action: None,
        ticks: 0,
    };
    for &step in steps {
        match step {
            Step::Tick => current.ticks += 1,
            Step::Drive(node, input) => {
                let next = scenario::Step {
                    action: Some(action(circuit, node, input)),
                    ticks: 0,
                };
                written.push(mem::replace(&mut current, next));
            }
        }
    }
    written.push(current);
    scenario::to_toml(Path::new(&format!("{seed}.ril")), written)
}

pub fn label(circuit: &Circuit, id: NodeId) -> String {
    target(circuit, id).to_string()
}

fn action(circuit: &Circuit, node: NodeId, input: Input) -> Action {
    Action {
        at: target(circuit, node),
        input,
    }
}

fn target(circuit: &Circuit, id: NodeId) -> Target {
    let node = circuit.node(id);
    match (node.names.first(), node.blocks.first()) {
        (Some(name), _) => Target::Name(name.clone()),
        (None, Some(&(pos, _))) => Target::from(pos),
        (None, None) => unreachable!("every node has a name or a block"),
    }
}
