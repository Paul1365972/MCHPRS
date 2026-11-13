pub mod reference;
#[cfg(test)]
mod tests;
pub mod trace;

use mchprs_redpiler::circuit::{Circuit, InputKinds, Link, NodeId};
use mchprs_redpiler::graph::{LinkKind, NodeKind, NodeState};
use mchprs_redpiler::nodes::{Changes, Nodes};
use mchprs_redstone::input::Input;
use mchprs_redstone::tick_queue::TickPriority;
use mchprs_simulation::ticks::SimulationError;
use mchprs_simulation::Simulation;
use reference::{sorted, Batch, Delivery, Queues, ReferenceEngine};
use rustc_hash::FxHashSet;
use std::iter::{self, Peekable};
use std::slice;
use trace::{PlayedNotes, Seen, Sight, View};

const STATE_BUDGET: usize = 20_000;

#[derive(Debug, Clone, Copy)]
pub enum Step {
    Drive(NodeId, Input),
    Tick,
}

pub enum Verdict {
    Passed,
    Skipped,
}

pub struct Illegal {
    pub steps_played: usize,
    pub seen: View,
    pub legal: Vec<View>,
}

/// Two runs with equal keys go on alike
#[derive(PartialEq, Eq, Hash)]
struct RunKey {
    states: Vec<NodeState>,
    queues: Queues,
    played_notes: PlayedNotes,
}

#[derive(Clone)]
struct Run {
    engine: ReferenceEngine,
    played_notes: PlayedNotes,
}

impl Run {
    fn new(circuit: &Circuit) -> Run {
        Run {
            engine: ReferenceEngine::new(circuit),
            played_notes: PlayedNotes::new(circuit),
        }
    }

    fn record_played_notes(&mut self) {
        let mut changes = Changes::default();
        let reported = self
            .engine
            .show(mchprs_simulation::View::Observed, &mut changes);
        reported.expect("the reference engine never fails");
        self.played_notes.record(&changes);
    }

    fn key(&mut self) -> RunKey {
        RunKey {
            states: self.engine.complete_snapshot().states,
            queues: self.engine.queues(),
            played_notes: self.played_notes.clone(),
        }
    }

    fn take_view(&mut self, observed: &[NodeId], sight: Sight, tick_ended: bool) -> View {
        let snapshot = self.engine.complete_snapshot();
        self.played_notes
            .take_view(snapshot, observed, sight, tick_ended)
    }
}

type DeliveryOrder = Vec<Delivery>;

pub struct Oracle<'a> {
    circuit: &'a Circuit,
    input_kinds: InputKinds,
    observed: &'a [NodeId],
    delivery_orders: Vec<Vec<DeliveryOrder>>,
}

impl<'a> Oracle<'a> {
    pub fn new(circuit: &'a Circuit, observed: &'a [NodeId]) -> Oracle<'a> {
        Oracle {
            circuit,
            input_kinds: circuit.input_kinds(),
            observed,
            delivery_orders: circuit
                .ids()
                .map(|node| delivery_orders(circuit, node))
                .collect(),
        }
    }

    pub fn check(&self, steps: &[Step], trace: &[Seen]) -> Result<Verdict, Illegal> {
        let mut trace = trace.iter().peekable();
        let first_played = trace.peek().map(|seen| seen.steps_played);
        assert_eq!(
            first_played,
            Some(0),
            "a trace starts with what was seen before any step"
        );
        let runs = vec![Run::new(self.circuit)];
        let mut runs = self.matching(runs, &[], &mut trace)?;
        for (index, &step) in steps.iter().enumerate() {
            runs = match step {
                Step::Drive(node, input) => self.after_input(runs, node, input),
                Step::Tick => match self.after_tick(runs) {
                    Some(ticked) => ticked,
                    None => return Ok(Verdict::Skipped),
                },
            };
            runs = self.matching(runs, &steps[..=index], &mut trace)?;
        }
        assert!(
            trace.next().is_none(),
            "a trace is in the order of `steps_played`, which is at most the number of steps"
        );
        Ok(Verdict::Passed)
    }

    fn matching(
        &self,
        runs: Vec<Run>,
        played: &[Step],
        trace: &mut Peekable<slice::Iter<'_, Seen>>,
    ) -> Result<Vec<Run>, Illegal> {
        let steps_played = played.len();
        let Some(seen) = trace.next_if(|seen| seen.steps_played == steps_played) else {
            return Ok(runs);
        };
        let tick_ended = matches!(played.last(), Some(Step::Tick));
        let mut matching = Vec::new();
        let mut legal = Vec::new();
        for mut run in runs {
            let view = run.take_view(self.observed, seen.view.sight(), tick_ended);
            let runs_pending_as_seen = match &seen.view.pending_ticks {
                Some(pending_ticks) => run.engine.run_pending_as(pending_ticks),
                None => true,
            };
            if view.nodes == seen.view.nodes && runs_pending_as_seen {
                matching.push(run);
            } else if !legal.contains(&view) {
                legal.push(view);
            }
        }
        match matching.is_empty() {
            true => Err(Illegal {
                steps_played,
                seen: seen.view.clone(),
                legal,
            }),
            false => Ok(matching),
        }
    }

    fn after_input(&self, runs: Vec<Run>, node: NodeId, input: Input) -> Vec<Run> {
        let input = self.input_kinds.accept(node, input);
        let input = input.expect("the steps drive inputs");
        let mut after = Vec::new();
        for run in runs {
            for order in &self.delivery_orders[node.index()] {
                let mut run = run.clone();
                run.engine.drive_in_order(node, input, order);
                run.record_played_notes();
                after.push(run);
            }
        }
        distinct(after)
    }

    fn after_tick(&self, mut runs: Vec<Run>) -> Option<Vec<Run>> {
        for run in &mut runs {
            run.engine.start_tick();
        }
        for priority in TickPriority::ALL {
            runs = self.after_priority(runs, priority)?;
        }
        Some(runs)
    }

    fn after_priority(&self, runs: Vec<Run>, priority: TickPriority) -> Option<Vec<Run>> {
        let mut unfinished: Vec<(Run, Vec<Batch>)> = Vec::new();
        for mut run in runs {
            let due = run.engine.take_due(priority);
            unfinished.push((run, due.into_iter().map(sorted).collect()));
        }
        let mut followed = FxHashSet::default();
        let mut finished = Vec::new();
        while let Some((mut run, mut due)) = unfinished.pop() {
            due.retain(|batch| !batch.is_empty());
            let Some(batch) = due.first() else {
                finished.push(run);
                continue;
            };
            if !followed.insert((run.key(), due.clone())) {
                continue;
            }
            if followed.len() > STATE_BUDGET {
                return None;
            }
            for (index, &node) in batch.iter().enumerate() {
                let mut still_due = due.clone();
                still_due[0].remove(index);
                for order in &self.delivery_orders[node.index()] {
                    let mut run = run.clone();
                    run.engine.tick_node(node, order);
                    run.record_played_notes();
                    unfinished.push((run, still_due.clone()));
                }
            }
        }
        Some(distinct(finished))
    }
}

pub fn play(simulation: &mut dyn Simulation<Nodes>, step: Step) -> Result<(), SimulationError> {
    match step {
        Step::Drive(node, input) => Ok(simulation.drive(node, input)?),
        Step::Tick => simulation.run(1).map(drop),
    }
}

fn distinct(runs: Vec<Run>) -> Vec<Run> {
    let mut keys = FxHashSet::default();
    let mut distinct = Vec::new();
    for mut run in runs {
        if keys.insert(run.key()) {
            distinct.push(run);
        }
    }
    distinct
}

/// Every order in which the node can deliver its links leaves a run with the key of one of these
fn delivery_orders(circuit: &Circuit, node: NodeId) -> Vec<DeliveryOrder> {
    let links = circuit.outputs(node);
    let mut orders: Vec<DeliveryOrder> = vec![links.iter().copied().map(Delivery::of).collect()];
    for target in targets_fed_through_both_inputs(links) {
        let updated_once_differs = matches!(circuit.node(target).kind, NodeKind::Comparator { .. });
        let mut next_orders = Vec::new();
        for order in orders {
            for first in [LinkKind::Default, LinkKind::Side] {
                let mut order = order.clone();
                order.sort_by_key(|delivery| !delivery.feeds(target, first));
                next_orders.push(order);
            }
            if updated_once_differs {
                let others = order
                    .into_iter()
                    .filter(|delivery| delivery.target() != target);
                let into_target = Delivery::into_target(target, links);
                next_orders.push(iter::once(into_target).chain(others).collect());
            }
        }
        orders = next_orders;
    }
    orders
}

fn targets_fed_through_both_inputs(links: &[Link]) -> Vec<NodeId> {
    let feeds = |target: NodeId, kind: LinkKind| {
        let mut links = links.iter();
        links.any(|link| link.target == target && link.kind == kind)
    };
    let mut targets: Vec<NodeId> = links
        .iter()
        .map(|link| link.target)
        .filter(|&target| feeds(target, LinkKind::Default) && feeds(target, LinkKind::Side))
        .collect();
    targets.sort_unstable();
    targets.dedup();
    targets
}
