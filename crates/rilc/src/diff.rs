use crate::numbers::{counted, grouped};
use crate::scenario::{Action, Location, Scenario};
use crate::setup::Setup;
use crate::subject::Subject;
use crate::{Error, Outcome};
use mchprs_redpiler::circuit::NodeId;
use mchprs_redpiler::engine::{Changes, Drive, Engine};
use mchprs_redpiler::graph::{LinkKind, NodeState};
use rustc_hash::FxHashMap;
use std::mem;
use std::path::Path;

const EXPLAINED_DIFFERENCES: usize = 12;

pub fn command(scenario_path: &Path, setups: [&Setup; 2]) -> Outcome {
    let scenario = Scenario::read(scenario_path)?;
    let input = scenario.read_input()?;
    let subjects = [
        Subject::compile(&input, setups[0])?,
        Subject::compile(&input, setups[1])?,
    ];
    let verdict = Pair::new(&subjects, setups).play(&scenario)?;
    print!("{}", verdict.report);
    Ok(verdict.first_difference_tick.is_none())
}

pub struct Verdict {
    pub first_difference_tick: Option<u64>,
    pub report: String,
}

pub struct Pair<'a> {
    subjects: &'a [Subject; 2],
    setups: [&'a Setup; 2],
    shared: Vec<(Location, [NodeId; 2])>,
    port_indices: Vec<usize>,
    unshared: [usize; 2],
}

impl<'a> Pair<'a> {
    pub fn new(subjects: &'a [Subject; 2], setups: [&'a Setup; 2]) -> Pair<'a> {
        let mut shared: Vec<_> = subjects[0]
            .nodes()
            .filter_map(|(location, first)| {
                let second = subjects[1].node_at(location)?;
                Some((location.clone(), [first, second]))
            })
            .collect();
        shared.sort();
        let port_indices = (0..shared.len())
            .filter(|&index| {
                let (_, ids) = &shared[index];
                (0..2).any(|setup| subjects[setup].circuit.node(ids[setup]).port.is_some())
            })
            .collect();
        let unshared = [0, 1].map(|setup| subjects[setup].nodes().count() - shared.len());
        Pair {
            subjects,
            setups,
            shared,
            port_indices,
            unshared,
        }
    }

    pub fn play(&self, scenario: &Scenario) -> Result<Verdict, Error> {
        let drives = [
            self.subjects[0].drives(scenario)?,
            self.subjects[1].drives(scenario)?,
        ];
        let mut engines = [
            self.setups[0].build(&self.subjects[0].circuit)?,
            self.setups[1].build(&self.subjects[1].circuit)?,
        ];
        let mut run = Run {
            pair: self,
            states: [engines[0].snapshot()?.states, engines[1].snapshot()?.states],
            previous_states: None,
            tick: 0,
            last_action: None,
            divergence: None,
            notes_played: 0,
        };
        let differences = run.play(scenario, &drives, &mut engines)?;
        let mut report = match &differences {
            Some(differences) => run.explain(differences),
            None => format!(
                "{} and {} agree on {} for {}, with {}\n",
                self.setups[0],
                self.setups[1],
                counted(self.port_indices.len() as u64, "port"),
                counted(run.tick, "tick"),
                counted(run.notes_played, "note play"),
            ),
        };
        let noun = if self.subjects[0].is_circuit() {
            "name"
        } else {
            "position"
        };
        report.push_str(&format!(
            "not compared: nodes at {} only {} has, at {} only {} has; queued ticks; changes \
             within a tick\n",
            counted(self.unshared[0] as u64, noun),
            self.setups[0],
            grouped(self.unshared[1] as u64),
            self.setups[1]
        ));
        Ok(Verdict {
            first_difference_tick: differences.map(|_| run.tick),
            report,
        })
    }
}

enum Difference {
    State(usize),
    Notes(Location, [u64; 2]),
}

struct Run<'a> {
    pair: &'a Pair<'a>,
    states: [Vec<NodeState>; 2],
    previous_states: Option<[Vec<NodeState>; 2]>,
    tick: u64,
    last_action: Option<(&'a Action, u64)>,
    divergence: Option<(u64, usize)>,
    notes_played: u64,
}

impl<'a> Run<'a> {
    fn play(
        &mut self,
        scenario: &'a Scenario,
        drives: &[Vec<Option<Drive>>; 2],
        engines: &mut [Box<dyn Engine>; 2],
    ) -> Result<Option<Vec<Difference>>, Error> {
        if let Some(differences) = self.compare(&FxHashMap::default()) {
            return Ok(Some(differences));
        }
        for (step_index, step) in scenario.steps.iter().enumerate() {
            if let Some(action) = &step.action {
                for (engine, drives) in engines.iter_mut().zip(drives) {
                    engine.drive(drives[step_index].expect("each action has a drive"))?;
                }
                self.last_action = Some((action, self.tick + 1));
            }
            for _ in 0..step.ticks {
                let mut notes = FxHashMap::default();
                let mut states = [Vec::new(), Vec::new()];
                for (setup, engine) in engines.iter_mut().enumerate() {
                    engine.run(1)?;
                    let mut changes = Changes::default();
                    engine.changes(&mut changes)?;
                    for (node, count) in changes.notes {
                        for location in self.pair.subjects[setup].locations(node) {
                            notes.entry(location).or_insert([0; 2])[setup] += u64::from(count);
                        }
                    }
                    states[setup] = engine.snapshot()?.states;
                }
                self.previous_states = Some(mem::replace(&mut self.states, states));
                self.tick += 1;
                if let Some(differences) = self.compare(&notes) {
                    return Ok(Some(differences));
                }
            }
        }
        Ok(None)
    }

    fn state(&self, setup: usize, shared: usize) -> NodeState {
        self.states[setup][self.pair.shared[shared].1[setup].index()]
    }

    fn differs(&self, shared: usize) -> bool {
        self.state(0, shared) != self.state(1, shared)
    }

    fn compare(&mut self, notes: &FxHashMap<Location, [u64; 2]>) -> Option<Vec<Difference>> {
        if self.divergence.is_none()
            && let Some(first) = (0..self.pair.shared.len()).find(|&index| self.differs(index))
        {
            self.divergence = Some((self.tick, first));
        }
        let mut differences: Vec<_> = self
            .pair
            .port_indices
            .iter()
            .copied()
            .filter(|&index| self.differs(index))
            .map(Difference::State)
            .collect();
        let mut played: Vec<_> = notes
            .iter()
            .filter(|(location, _)| {
                self.pair
                    .subjects
                    .iter()
                    .all(|subject| subject.node_at(location).is_some())
            })
            .collect();
        played.sort();
        for (location, counts) in played {
            if counts[0] != counts[1] {
                differences.push(Difference::Notes(location.clone(), *counts));
            } else {
                self.notes_played += counts[0];
            }
        }
        (!differences.is_empty()).then_some(differences)
    }

    fn explain(&self, differences: &[Difference]) -> String {
        let [first, second] = self.pair.setups;
        let when = |tick| match tick {
            0 => "before the first tick".to_owned(),
            tick => format!("after tick {}", grouped(tick)),
        };
        let mut report = format!("{first} and {second} differ {}\n", when(self.tick));
        for difference in differences.iter().take(EXPLAINED_DIFFERENCES) {
            match difference {
                Difference::State(index) => self.explain_state(&mut report, *index),
                Difference::Notes(location, counts) => report.push_str(&format!(
                    "notes at {location}: {first} played {}, {second} played {}\n",
                    grouped(counts[0]),
                    grouped(counts[1])
                )),
            }
        }
        if differences.len() > EXPLAINED_DIFFERENCES {
            let more = (differences.len() - EXPLAINED_DIFFERENCES) as u64;
            report.push_str(&format!("and {} more\n", counted(more, "difference")));
        }
        match self.last_action {
            Some((action, tick)) => report.push_str(&format!(
                "last action: {action} before tick {}\n",
                grouped(tick)
            )),
            None => report.push_str("last action: none\n"),
        }
        if let Some((tick, index)) = self.divergence {
            let location = &self.pair.shared[index].0;
            report.push_str(&format!(
                "the shared nodes first differed {}, at {location}\n",
                when(tick)
            ));
        }
        report
    }

    fn explain_state(&self, report: &mut String, index: usize) {
        let setups = self.pair.setups;
        let (location, ids) = &self.pair.shared[index];
        let [first, second] = [0, 1].map(|setup| self.component(setup, ids[setup], &self.states));
        report.push_str(&format!(
            "{location}: {} {first}, {} {second}\n",
            setups[0], setups[1]
        ));
        for setup in 0..2 {
            let subject = &self.pair.subjects[setup];
            let inputs = subject.circuit.inputs(ids[setup]);
            if inputs.is_empty() {
                report.push_str(&format!("  inputs under {}: none\n", setups[setup]));
                continue;
            }
            report.push_str(&format!("  inputs under {}:\n", setups[setup]));
            for link in inputs {
                let kind = match link.kind {
                    LinkKind::Default => "default",
                    LinkKind::Side => "side",
                };
                let source = subject.locations(link.source).min().map_or_else(
                    || format!("node {}", link.source.index()),
                    |location| location.to_string(),
                );
                let now = self.component(setup, link.source, &self.states);
                let state = match &self.previous_states {
                    Some(previous) => {
                        let before = self.component(setup, link.source, previous);
                        if before == now {
                            now
                        } else {
                            format!("{before} -> {now}")
                        }
                    }
                    None => now,
                };
                report.push_str(&format!(
                    "    {kind} link of weight {} from {source}: {state}\n",
                    link.weight
                ));
            }
        }
    }

    fn component(&self, setup: usize, id: NodeId, states: &[Vec<NodeState>; 2]) -> String {
        let node = self.pair.subjects[setup].circuit.node(id);
        mchprs_ril::component(&node.kind, states[setup][id.index()]).to_string()
    }
}
