use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::behavior::{InputStrengths, Outcome, Pending};
use mchprs_redpiler::circuit::{Circuit, InputKinds, Link, NodeId};
use mchprs_redpiler::graph::NodeState;
use mchprs_redpiler::nodes::{Changes, Nodes, Snapshot};
use mchprs_redstone::input::Input;
use mchprs_redstone::tick_queue::{PendingTick, TickPriority};
use mchprs_simulation::input::DriveError;
use mchprs_simulation::readback::Readback;
use mchprs_simulation::stepper;
use mchprs_simulation::ticks::{settle_until_hit, Ran, SimulationError};
use mchprs_simulation::watch::Watch;
use mchprs_simulation::{Capabilities, End, Running, Simulation, View};
use std::collections::{BTreeMap, BTreeSet};
use std::mem;

#[derive(Debug)]
pub struct PhaseEngine {
    circuit: Circuit,
    input_kinds: InputKinds,
    states: Vec<NodeState>,
    due_ticks: Vec<Option<u64>>,
    queues_by_due_tick: BTreeMap<(u64, TickPriority), Vec<NodeId>>,
    ticks: u64,
    changed: Vec<bool>,
    played_note: Vec<bool>,
}

impl PhaseEngine {
    pub fn new(circuit: &Circuit) -> PhaseEngine {
        let initial = circuit.initial();
        let mut engine = PhaseEngine {
            circuit: circuit.clone(),
            input_kinds: circuit.input_kinds(),
            states: initial.states,
            due_ticks: vec![None; circuit.len()],
            queues_by_due_tick: BTreeMap::new(),
            ticks: 0,
            changed: vec![false; circuit.len()],
            played_note: vec![false; circuit.len()],
        };
        for (node, tick) in initial.pending_ticks {
            engine.schedule(node, tick);
        }
        engine
    }

    pub fn complete_snapshot(&self) -> Snapshot {
        let queues = self.queues_by_due_tick.iter();
        let pending_ticks = queues.flat_map(|(&(due, priority), nodes)| {
            let delay = (due - self.ticks) as u32;
            let tick = PendingTick { delay, priority };
            nodes.iter().map(move |&node| (node, tick))
        });
        Snapshot {
            states: self.states.clone(),
            pending_ticks: pending_ticks.collect(),
        }
    }

    fn run_tick(&mut self) {
        self.ticks += 1;
        for priority in TickPriority::ALL {
            let due = self.queues_by_due_tick.remove(&(self.ticks, priority));
            let due = due.unwrap_or_default();
            let outcomes: Vec<_> = due
                .iter()
                .map(|&node| (node, self.tick_outcome(node)))
                .collect();
            for node in due {
                self.due_ticks[node.index()].take_if(|due| *due == self.ticks);
            }
            self.step(outcomes);
        }
    }

    fn step(&mut self, outcomes: Vec<(NodeId, Outcome)>) {
        let mut reached = BTreeSet::new();
        for (node, outcome) in outcomes {
            let old = self.states[node.index()].power;
            let new = outcome.state.power;
            self.set(node, outcome);
            let links = self.circuit.outputs(node).iter();
            let changed = links.filter(|link| carried(old, link) != carried(new, link));
            reached.extend(changed.map(|link| link.target));
        }
        let updated: Vec<_> = reached
            .into_iter()
            .map(|node| (node, self.update_outcome(node)))
            .collect();
        for (node, outcome) in updated {
            debug_assert!(
                self.circuit.outputs(node).is_empty()
                    || outcome.state.power == self.states[node.index()].power,
                "an update changes the power of a node that has outgoing links"
            );
            self.set(node, outcome);
        }
    }

    fn tick_outcome(&self, node: NodeId) -> Outcome {
        let kind = &self.circuit.node(node).kind;
        kind.tick(self.states[node.index()], self.inputs(node))
    }

    fn update_outcome(&self, node: NodeId) -> Outcome {
        let kind = &self.circuit.node(node).kind;
        let state = self.states[node.index()];
        kind.update(state, self.inputs(node), self.pending(node))
    }

    fn inputs(&self, node: NodeId) -> InputStrengths {
        let mut inputs = InputStrengths::default();
        for link in self.circuit.inputs(node) {
            let power = self.states[link.source.index()].power;
            inputs.raise(link.kind, carried(power, link));
        }
        inputs
    }

    fn pending(&self, node: NodeId) -> Pending {
        match self.due_ticks[node.index()] {
            None => Pending::None,
            Some(due) if due == self.ticks => Pending::DueThisTick,
            Some(_) => Pending::Later,
        }
    }

    fn set(&mut self, node: NodeId, outcome: Outcome) {
        let index = node.index();
        let old = mem::replace(&mut self.states[index], outcome.state);
        self.changed[index] |= old.shown() != outcome.state.shown();
        if let Some(tick) = outcome.schedule {
            self.schedule(node, tick);
        }
        self.played_note[index] |= outcome.plays_note;
    }

    fn schedule(&mut self, node: NodeId, tick: PendingTick) {
        let due = self.ticks + u64::from(tick.delay);
        let queue = self.queues_by_due_tick.entry((due, tick.priority));
        queue.or_default().push(node);
        self.due_ticks[node.index()] = Some(due);
    }

    fn is_idle(&self) -> bool {
        self.queues_by_due_tick.is_empty()
    }
}

impl Simulation<Nodes> for PhaseEngine {
    fn capabilities(&self) -> Capabilities {
        Capabilities {
            breakpoints: false,
            settling: true,
            write_back: true,
        }
    }

    fn ticks(&self) -> u64 {
        self.ticks
    }

    fn run(&mut self, ticks: u64) -> Result<Ran, SimulationError> {
        for _ in 0..ticks {
            self.run_tick();
        }
        Ok(Ran { ticks, stop: None })
    }

    fn settle(&mut self, limit: u64) -> Result<Ran, SimulationError> {
        Ok(settle_until_hit(self, limit, Self::is_idle, |engine| {
            engine.run_tick();
            None
        }))
    }

    fn drive(&mut self, node: NodeId, input: Input) -> Result<(), DriveError> {
        let input = self.input_kinds.accept(node, input)?;
        let driven = input.drive(self.states[node.index()], self.pending(node));
        self.step(vec![(node, driven)]);
        Ok(())
    }

    fn watch(&mut self, _: Watch<NodeId>) -> Result<(), SimulationError> {
        Ok(())
    }

    fn show(&mut self, view: View, changes: &mut Changes) -> Result<(), SimulationError> {
        for node in self.circuit.ids() {
            let index = node.index();
            if view == View::Observed && !self.circuit.node(node).observed {
                continue;
            }
            if mem::take(&mut self.changed[index]) {
                changes.states.push((node, self.states[index]));
            }
            if mem::take(&mut self.played_note[index]) {
                changes.played_note_blocks.push(node);
            }
        }
        Ok(())
    }

    fn read(&mut self) -> Result<Readback<Snapshot>, SimulationError> {
        Ok(Readback::Complete(self.complete_snapshot()))
    }

    fn stop(mut self: Box<Self>) -> Result<End<Nodes>, SimulationError> {
        let mut changes = Changes::default();
        self.show(View::Observed, &mut changes)?;
        Ok(End {
            changes,
            state: Readback::Complete(self.complete_snapshot()),
        })
    }

    fn start(self: Box<Self>) -> Result<Box<dyn Running<Nodes>>, SimulationError> {
        stepper::start(self, "phase")
    }
}

fn carried(power: SignalStrength, link: &Link) -> SignalStrength {
    power.saturating_sub(link.weight)
}
