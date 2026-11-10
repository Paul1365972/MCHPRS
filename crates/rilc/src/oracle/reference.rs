use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::backend::{GraphBackend, GraphContext};
use mchprs_redpiler::behavior::AcceptedInput;
use mchprs_redpiler::behavior::{InputStrengths, Outcome, Pending};
use mchprs_redpiler::circuit::{Circuit, InputKinds, Link, NodeId};
use mchprs_redpiler::extract::ExtractOptions;
use mchprs_redpiler::graph::NodeState;
use mchprs_redpiler::graph_simulation;
use mchprs_redpiler::nodes::{Changes, Nodes, Snapshot};
use mchprs_redstone::input::Input;
use mchprs_redstone::tick_queue::{PendingTick, TickPriority};
use mchprs_ril::extraction;
use mchprs_simulation::backend::{Backend, CompileContext, CompileError, CompileStep, Source};
use mchprs_simulation::input::DriveError;
use mchprs_simulation::layout::Layout;
use mchprs_simulation::readback::Readback;
use mchprs_simulation::stepper;
use mchprs_simulation::ticks::{settle_until_hit, Ran, SimulationError};
use mchprs_simulation::watch::Watch;
use mchprs_simulation::{Blocks, Capabilities, End, Running, Simulation, View};
use mchprs_syntax::flag::{Flag, FlagValues};
use std::borrow::Borrow;
use std::collections::BTreeMap;
use std::mem;
use std::sync::Arc;

#[derive(Debug, Clone, Copy, Default)]
pub struct ReferenceBackend;

impl Backend for ReferenceBackend {
    fn name(&self) -> &str {
        "reference"
    }

    fn flags(&self) -> &[Flag] {
        &extraction::FLAGS
    }

    fn compile(
        &self,
        source: &Source<'_>,
        flags: &FlagValues,
        context: &mut CompileContext<'_>,
    ) -> Result<(Box<dyn Simulation<Blocks>>, Layout), CompileError> {
        graph_simulation::compile(self, source, flags, context)
    }
}

impl GraphBackend for ReferenceBackend {
    fn extract_options(&self, flags: &FlagValues) -> ExtractOptions {
        extraction::options(flags)
    }

    fn build(
        &self,
        circuit: &Circuit,
        _: &FlagValues,
        context: &mut GraphContext<'_>,
    ) -> Result<Box<dyn Simulation<Nodes>>, CompileError> {
        context.report(CompileStep::single("Building the engine"))?;
        Ok(Box::new(ReferenceEngine::new(circuit)))
    }
}

#[derive(Debug, Clone)]
pub(super) struct Delivery {
    target: NodeId,
    links: Vec<Link>,
}

impl Delivery {
    pub(super) fn of(link: Link) -> Delivery {
        Delivery {
            target: link.target,
            links: vec![link],
        }
    }
}

/// Ticks that run in any order: those one change schedules into one queue by updating its targets,
/// or the one a node schedules for itself.
pub(super) type Batch = Vec<NodeId>;

/// The batches of a queue are in the order they were scheduled
pub(super) type Queues = BTreeMap<(u64, TickPriority), Vec<Batch>>;

#[derive(Debug, Clone)]
pub struct ReferenceEngine {
    circuit: Arc<Circuit>,
    input_kinds: Arc<InputKinds>,
    states: Vec<NodeState>,
    input_link_strengths: Vec<Vec<SignalStrength>>,
    due_ticks: Vec<Option<u64>>,
    queues_by_due_tick: Queues,
    /// The queues whose last batch takes the next tick scheduled into them
    batching_queues: Vec<(u64, TickPriority)>,
    tick: u64,
    changed: Vec<bool>,
    has_unreported_play: Vec<bool>,
}

impl ReferenceEngine {
    pub fn new(circuit: &Circuit) -> ReferenceEngine {
        let initial = circuit.initial();
        let strength_of = |link: &Link| {
            let power = initial.states[link.source.index()].power;
            power.saturating_sub(link.weight)
        };
        let input_link_strengths = circuit
            .ids()
            .map(|node| circuit.inputs(node).iter().map(strength_of).collect())
            .collect();
        let mut engine = ReferenceEngine {
            circuit: Arc::new(circuit.clone()),
            input_kinds: Arc::new(circuit.input_kinds()),
            input_link_strengths,
            due_ticks: vec![None; circuit.len()],
            queues_by_due_tick: BTreeMap::new(),
            batching_queues: Vec::new(),
            tick: 0,
            changed: vec![false; circuit.len()],
            has_unreported_play: vec![false; circuit.len()],
            states: initial.states,
        };
        for (node, tick) in initial.pending_ticks {
            engine.schedule(node, tick);
            engine.batching_queues.clear();
        }
        engine
    }

    /// A tick is `start_tick`, then for each priority of [`TickPriority::ALL`] in turn one
    /// `tick_node` per node of `take_due`.
    pub(super) fn start_tick(&mut self) {
        self.tick += 1;
    }

    pub(super) fn take_due(&mut self, priority: TickPriority) -> Vec<Batch> {
        self.queues_by_due_tick
            .remove(&(self.tick, priority))
            .unwrap_or_default()
    }

    pub fn complete_snapshot(&self) -> Snapshot {
        let queues = self.queues_by_due_tick.iter();
        let pending_ticks = queues.flat_map(|(&(due, priority), batches)| {
            let tick = PendingTick {
                delay: u32::try_from(due - self.tick).expect("delays fit u32"),
                priority,
            };
            batches.iter().flatten().map(move |&node| (node, tick))
        });
        Snapshot {
            states: self.states.clone(),
            pending_ticks: pending_ticks.collect(),
        }
    }

    /// `order` delivers every outgoing link of the node once
    pub(super) fn tick_node(
        &mut self,
        node: NodeId,
        order: impl IntoIterator<Item = impl Borrow<Delivery>>,
    ) {
        if self.due_ticks[node.index()] == Some(self.tick) {
            self.due_ticks[node.index()] = None;
        }
        let kind = self.circuit.node(node).kind;
        let ticked = kind.tick(self.states[node.index()], self.inputs(node));
        self.set_and_update_targets(node, ticked, order);
    }

    pub(super) fn drive_in_order(
        &mut self,
        node: NodeId,
        input: AcceptedInput,
        order: impl IntoIterator<Item = impl Borrow<Delivery>>,
    ) {
        let driven = input.drive(self.states[node.index()], self.pending(node));
        self.set_and_update_targets(node, driven, order);
    }

    fn inputs(&self, node: NodeId) -> InputStrengths {
        let mut inputs = InputStrengths::default();
        let links = self.circuit.inputs(node).iter();
        for (link, &strength) in links.zip(&self.input_link_strengths[node.index()]) {
            inputs.raise(link.kind, strength);
        }
        inputs
    }

    fn pending(&self, node: NodeId) -> Pending {
        match self.due_ticks[node.index()] {
            None => Pending::None,
            Some(due) if due == self.tick => Pending::DueThisTick,
            Some(_) => Pending::Later,
        }
    }

    fn schedule(&mut self, node: NodeId, tick: PendingTick) {
        let due = self.tick + u64::from(tick.delay);
        let queue = (due, tick.priority);
        let batches = self.queues_by_due_tick.entry(queue).or_default();
        match batches.last_mut() {
            Some(batch) if self.batching_queues.contains(&queue) => batch.push(node),
            _ => {
                batches.push(vec![node]);
                self.batching_queues.push(queue);
            }
        }
        self.due_ticks[node.index()] = Some(due);
    }

    fn set(&mut self, node: NodeId, outcome: Outcome) {
        if let Some(tick) = outcome.schedule {
            self.schedule(node, tick);
        }
        self.has_unreported_play[node.index()] |= outcome.plays_note;
        let state = &mut self.states[node.index()];
        self.changed[node.index()] |= state.shown() != outcome.state.shown();
        *state = outcome.state;
    }

    fn set_and_update_targets(
        &mut self,
        node: NodeId,
        outcome: Outcome,
        order: impl IntoIterator<Item = impl Borrow<Delivery>>,
    ) {
        let old = self.states[node.index()].power;
        let new = outcome.state.power;
        self.set(node, outcome);
        self.batching_queues.clear();
        if old == new {
            return;
        }
        for delivery in order {
            let Delivery { target, ref links } = *delivery.borrow();
            let mut changed = false;
            for link in links {
                let old_strength = old.saturating_sub(link.weight);
                let new_strength = new.saturating_sub(link.weight);
                if old_strength == new_strength {
                    continue;
                }
                let strengths = &mut self.input_link_strengths[target.index()];
                let mut incoming = self.circuit.inputs(target).iter().zip(strengths);
                let (_, strength) = incoming
                    .find(|(incoming, strength)| *incoming == link && **strength == old_strength)
                    .expect("the target has the link among its inputs");
                *strength = new_strength;
                changed = true;
            }
            if changed {
                let updated = self.circuit.node(target).kind.update(
                    self.states[target.index()],
                    self.inputs(target),
                    self.pending(target),
                );
                self.set(target, updated);
            }
        }
        self.batching_queues.clear();
    }

    fn run_tick(&mut self) {
        let circuit = Arc::clone(&self.circuit);
        self.start_tick();
        for priority in TickPriority::ALL {
            for node in self.take_due(priority).into_iter().flatten() {
                let outputs = circuit.outputs(node).iter().copied();
                self.tick_node(node, outputs.map(Delivery::of));
            }
        }
    }

    fn is_idle(&self) -> bool {
        self.queues_by_due_tick.is_empty()
    }
}

impl Simulation<Nodes> for ReferenceEngine {
    fn capabilities(&self) -> Capabilities {
        Capabilities {
            breakpoints: false,
            settling: true,
            write_back: true,
        }
    }

    fn ticks(&self) -> u64 {
        self.tick
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
        let circuit = Arc::clone(&self.circuit);
        let outputs = circuit.outputs(node).iter().copied();
        self.drive_in_order(node, input, outputs.map(Delivery::of));
        Ok(())
    }

    fn watch(&mut self, _: Watch<NodeId>) -> Result<(), SimulationError> {
        Ok(())
    }

    fn show(&mut self, view: View, changes: &mut Changes) -> Result<(), SimulationError> {
        for node in self.circuit.ids() {
            if view == View::Observed && !self.circuit.node(node).observed {
                continue;
            }
            if mem::take(&mut self.changed[node.index()]) {
                changes.states.push((node, self.states[node.index()]));
            }
            if mem::take(&mut self.has_unreported_play[node.index()]) {
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
        stepper::start(self, "reference")
    }
}
