use crate::numbers::{counted, grouped, when};
use crate::runner::{describe, input_slots, start, Run};
use crate::scenario::Scenario;
use crate::setup::Setup;
use crate::{Error, Outcome};
use mchprs_redstone::sound::SoundSource;
use rustc_hash::FxHashSet;
use std::cmp::Ordering;
use std::collections::BTreeMap;
use std::path::Path;

const SHOWN_DIFFERENCES: usize = 12;

pub struct Played {
    pub components: usize,
    pub ticks: u64,
    pub divergence: Option<String>,
}

pub fn command(scenario_path: &Path, setups: [&Setup; 2], compare_every: CompareEvery) -> Outcome {
    let scenario = Scenario::read(scenario_path)?;
    let input = scenario.read_input()?;
    let mut runs = [start(&input, setups[0])?, start(&input, setups[1])?];
    let played = play(&mut runs, &scenario, setups, compare_every)?;
    let [first, second] = setups;
    match played.divergence {
        None => {
            println!(
                "{first} and {second} agree on {} for {}",
                counted(played.components as u64, "component"),
                counted(played.ticks, "tick")
            );
            Ok(true)
        }
        Some(divergence) => {
            println!("{first} and {second} differ {}", when(played.ticks));
            println!("{divergence}");
            Ok(false)
        }
    }
}

#[derive(Clone, Copy)]
pub enum CompareEvery {
    Tick,
    Step,
}

pub fn play(
    runs: &mut [Box<dyn Run>; 2],
    scenario: &Scenario,
    setups: [&Setup; 2],
    compare_every: CompareEvery,
) -> Result<Played, Error> {
    let slots = [
        input_slots(&mut *runs[0], scenario)?,
        input_slots(&mut *runs[1], scenario)?,
    ];
    let pairs = pairs([&*runs[0], &*runs[1]]);
    let mut ticks = 0;
    let mut last_action = None;
    let mut differences = compare(runs, &pairs, setups)?;
    'steps: for (index, step) in scenario.steps.iter().enumerate() {
        if !differences.is_empty() {
            break;
        }
        if let Some(action) = &step.action {
            for (run, slots) in runs.iter_mut().zip(&slots) {
                let slot = slots[index].expect("every action has a slot");
                run.drive(slot, action.input)
                    .map_err(|error| scenario.step_error(index, error))?;
            }
            last_action = Some((action, ticks + 1));
        }
        let chunk = match compare_every {
            CompareEvery::Tick => 1,
            CompareEvery::Step => step.ticks.max(1),
        };
        let end = ticks + step.ticks;
        while ticks < end {
            let run_ticks = chunk.min(end - ticks);
            for run in runs.iter_mut() {
                run.run(run_ticks)?;
            }
            ticks += run_ticks;
            differences = compare(runs, &pairs, setups)?;
            if !differences.is_empty() {
                break 'steps;
            }
        }
    }
    let divergence = (!differences.is_empty()).then(|| {
        let mut lines: Vec<String> = differences
            .iter()
            .take(SHOWN_DIFFERENCES)
            .cloned()
            .collect();
        if differences.len() > SHOWN_DIFFERENCES {
            let more = (differences.len() - SHOWN_DIFFERENCES) as u64;
            lines.push(format!("and {} more", counted(more, "difference")));
        }
        lines.push(match last_action {
            Some((action, tick)) => format!("last action: {action} before tick {}", grouped(tick)),
            None => "last action: none".to_owned(),
        });
        lines.join("\n")
    });
    Ok(Played {
        components: pairs.len(),
        ticks,
        divergence,
    })
}

fn pairs(runs: [&dyn Run; 2]) -> Vec<[usize; 2]> {
    let [first, second] = runs.map(|run| run.targets());
    let (mut i, mut j) = (0, 0);
    let mut paired = FxHashSet::default();
    let mut pairs = Vec::new();
    while i < first.len() && j < second.len() {
        match first[i].cmp(&second[j]) {
            Ordering::Less => i += 1,
            Ordering::Greater => j += 1,
            Ordering::Equal => {
                if paired.insert([runs[0].component(i), runs[1].component(j)]) {
                    pairs.push([i, j]);
                }
                i += 1;
                j += 1;
            }
        }
    }
    pairs
}

fn compare(
    runs: &mut [Box<dyn Run>; 2],
    pairs: &[[usize; 2]],
    setups: [&Setup; 2],
) -> Result<Vec<String>, Error> {
    let [first, second] = setups;
    let mut differences = Vec::new();
    for &[first_slot, second_slot] in pairs {
        let shown = [runs[0].shown(first_slot)?, runs[1].shown(second_slot)?];
        let [a, b] = shown.map(|shown| shown.map(|(_, state)| state));
        if a != b {
            let target = &runs[0].targets()[first_slot];
            let [a, b] = shown.map(describe);
            differences.push(format!("{target}: {first} {a}, {second} {b}"));
        }
    }
    let mut played: BTreeMap<usize, [bool; 2]> = BTreeMap::new();
    for (side, run) in runs.iter_mut().enumerate() {
        for (slot, sound) in run.take_sounds()? {
            let pair = pairs.binary_search_by_key(&slot, |pair| pair[side]);
            if let Ok(pair) = pair
                && sound.source == SoundSource::Records
            {
                played.entry(pair).or_default()[side] = true;
            }
        }
    }
    for (pair, played) in played {
        let only = match played {
            [true, false] => first,
            [false, true] => second,
            _ => continue,
        };
        let target = &runs[0].targets()[pairs[pair][0]];
        differences.push(format!("note at {target}: only {only} played it"));
    }
    Ok(differences)
}
