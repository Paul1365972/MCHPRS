#[cfg(test)]
mod spec_tests;
mod tick;
mod update;
mod watching;

use crate::lower;
use crate::node::{ForwardLinks, Node, Nodes, Slot};
use crate::scheduler::{check_delays, TickScheduler};
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::behavior::Outcome;
use mchprs_redpiler::circuit::{Circuit, InputKinds, NodeId};
use mchprs_redpiler::nodes::{self, Changes, Snapshot};
use mchprs_redstone::input::Input;
use mchprs_redstone::tick_queue::{PendingTick, TickPriority};
use mchprs_simulation::backend::CompileError;
use mchprs_simulation::input::DriveError;
use mchprs_simulation::readback::Readback;
use mchprs_simulation::stepper;
use mchprs_simulation::ticks::{run_until_hit, settle_until_hit, Ran, SimulationError, Stop};
use mchprs_simulation::watch::{Hit, Watch};
use mchprs_simulation::{Capabilities, End, Running, Simulation, View};
use std::mem;
use watching::Watching;

#[derive(Debug)]
pub struct DirectEngine {
    input_kinds: InputKinds,
    nodes: Nodes,
    observed_nodes: Box<[Slot]>,
    forward_links: ForwardLinks,
    scheduler: TickScheduler,
    ticks_run: u64,
    watching: Option<Box<Watching>>,
}

impl DirectEngine {
    pub fn new(circuit: &Circuit) -> Result<DirectEngine, CompileError> {
        let (nodes, forward_links) = lower::lower(circuit)?;
        let initial = circuit.initial();
        check_delays(&initial.pending_ticks)
            .map_err(|error| CompileError::Unsupported(error.to_string()))?;
        let observed_nodes = circuit
            .ids()
            .filter(|&id| circuit.node(id).observed)
            .map(|id| nodes.get(id.index()))
            .collect();
        let mut engine = DirectEngine {
            input_kinds: circuit.input_kinds(),
            nodes,
            observed_nodes,
            forward_links,
            scheduler: TickScheduler::default(),
            ticks_run: 0,
            watching: None,
        };
        engine.init_inputs();
        engine.schedule(&initial.pending_ticks);
        Ok(engine)
    }

    fn init_inputs(&mut self) {
        for index in 0..self.nodes.inner().len() {
            let source = &self.nodes.inner()[index];
            let (power, links) = (source.state.power, source.forward_links.clone());
            for link in self.forward_links.get(&links) {
                let weakened = power.saturating_sub(link.weight());
                let target = &mut self.nodes[link.node()];
                let inputs = if link.side() {
                    &mut target.side_inputs
                } else {
                    &mut target.default_inputs
                };
                inputs.update_power(SignalStrength::Zero, weakened);
            }
        }
    }

    fn schedule(&mut self, pending_ticks: &[(NodeId, PendingTick)]) {
        for &(node, tick) in pending_ticks {
            let slot = self.nodes.get(node.index());
            let node = &mut self.nodes[slot];
            schedule_tick(&mut self.scheduler, slot, node, tick.delay, tick.priority);
        }
    }

    #[inline(always)]
    fn tick<const WATCHED: bool>(&mut self) {
        let queues = self.scheduler.queues_this_tick();
        queues.for_each(|slot| {
            self.tick_node::<WATCHED>(slot);
        });

        self.scheduler.end_tick(queues);
    }

    fn is_idle(&self) -> bool {
        !self.scheduler.has_pending_ticks()
    }

    fn apply<const WATCHED: bool>(&mut self, slot: Slot, outcome: Outcome) {
        let node = &mut self.nodes[slot];
        if let Some(tick) = outcome.schedule {
            schedule_tick(&mut self.scheduler, slot, node, tick.delay, tick.priority);
        }
        if outcome.state.repeater_locked != node.state.repeater_locked {
            node.set_repeater_locked(outcome.state.repeater_locked);
        }
        node.state.plate_touched = outcome.state.plate_touched;
        node.played_note |= outcome.plays_note;
        if outcome.state.power != self.nodes[slot].state.power {
            self.set_power_and_propagate::<WATCHED>(slot, outcome.state.power);
        }
    }

    fn set_power_and_propagate<const WATCHED: bool>(&mut self, slot: Slot, power: SignalStrength) {
        let node = &mut self.nodes[slot];
        let old_power = node.state.power;
        node.set_power(power);
        if WATCHED
            && old_power.is_zero() != power.is_zero()
            && let Some(watching) = &mut self.watching
        {
            watching.notice(slot);
        }

        for forward_link in self.forward_links.get(&node.forward_links) {
            let side = forward_link.side();
            let weight = forward_link.weight();
            let update = forward_link.node();

            let update_ref = &mut self.nodes[update];
            let inputs = if side {
                &mut update_ref.side_inputs
            } else {
                &mut update_ref.default_inputs
            };

            let old_input = old_power.saturating_sub(weight);
            let new_input = power.saturating_sub(weight);

            if old_input == new_input {
                continue;
            }

            inputs.update_power(old_input, new_input);

            let was_powered = WATCHED && self.nodes[update].is_powered();
            update::update_node(&mut self.scheduler, &mut self.nodes, update);
            if WATCHED
                && self.nodes[update].is_powered() != was_powered
                && let Some(watching) = &mut self.watching
            {
                watching.notice(update);
            }
        }
    }

    #[inline(never)]
    fn run_watched(&mut self, ticks: u64) -> Ran {
        run_until_hit(self, ticks, DirectEngine::tick_watched)
    }

    #[inline(never)]
    fn settle_watched(&mut self, limit: u64) -> Ran {
        settle_until_hit(
            self,
            limit,
            DirectEngine::is_idle,
            DirectEngine::tick_watched,
        )
    }

    fn tick_watched(&mut self) -> Option<Hit> {
        self.tick::<true>();
        self.ticks_run += 1;
        let watching = self.watching.as_deref_mut()?;
        watching.end_tick(self.ticks_run, &self.nodes)
    }
}

impl Simulation<nodes::Nodes> for DirectEngine {
    fn capabilities(&self) -> Capabilities {
        Capabilities {
            breakpoints: true,
            settling: true,
            write_back: true,
        }
    }

    fn ticks(&self) -> u64 {
        self.ticks_run
    }

    fn run(&mut self, ticks: u64) -> Result<Ran, SimulationError> {
        if self.watching.is_some() {
            return Ok(self.run_watched(ticks));
        }
        self.ticks_run += ticks;
        for _ in 0..ticks {
            self.tick::<false>();
        }
        Ok(Ran { ticks, stop: None })
    }

    fn settle(&mut self, limit: u64) -> Result<Ran, SimulationError> {
        if self.watching.is_some() {
            return Ok(self.settle_watched(limit));
        }
        let mut ticks = 0;
        while !self.is_idle() && ticks < limit {
            self.tick::<false>();
            ticks += 1;
        }
        self.ticks_run += ticks;
        Ok(Ran {
            ticks,
            stop: self.is_idle().then_some(Stop::Settled),
        })
    }

    fn drive(&mut self, node: NodeId, input: Input) -> Result<(), DriveError> {
        let input = self.input_kinds.accept(node, input)?;
        let slot = self.nodes.get(node.index());
        let node = &self.nodes[slot];
        let pending = node.pending(self.scheduler.current_tick());
        let outcome = input.drive(node.state, pending);
        if self.watching.is_some() {
            self.apply::<true>(slot, outcome);
        } else {
            self.apply::<false>(slot, outcome);
        }
        Ok(())
    }

    fn watch(&mut self, watch: Watch<NodeId>) -> Result<(), SimulationError> {
        let previous = self.watching.take();
        self.watching = Watching::new(watch, previous.as_deref(), &self.nodes).map(Box::new);
        Ok(())
    }

    fn show(&mut self, view: View, changes: &mut Changes) -> Result<(), SimulationError> {
        let all: Vec<Slot>;
        let slots = match view {
            View::Observed => &self.observed_nodes[..],
            View::All => {
                all = (0..self.nodes.inner().len())
                    .map(|index| self.nodes.get(index))
                    .collect();
                &all
            }
        };
        for &slot in slots {
            let node = &mut self.nodes[slot];
            let id = NodeId::new(slot.index());
            if mem::take(&mut node.changed) {
                changes.states.push((id, node.state));
            }
            if mem::take(&mut node.played_note) {
                changes.played_note_blocks.push(id);
            }
        }
        Ok(())
    }

    fn read(&mut self) -> Result<Readback<Snapshot>, SimulationError> {
        Ok(Readback::Complete(Snapshot {
            states: self.nodes.inner().iter().map(|node| node.state).collect(),
            pending_ticks: self
                .scheduler
                .pending_ticks()
                .map(|(node, tick)| (NodeId::new(node.index()), tick))
                .collect(),
        }))
    }

    fn stop(mut self: Box<Self>) -> Result<End<nodes::Nodes>, SimulationError> {
        let mut changes = Changes::default();
        self.show(View::Observed, &mut changes)?;
        Ok(End {
            changes,
            state: self.read()?,
        })
    }

    fn start(self: Box<Self>) -> Result<Box<dyn Running<nodes::Nodes>>, SimulationError> {
        stepper::start(self, "direct")
    }
}

fn schedule_tick(
    scheduler: &mut TickScheduler,
    slot: Slot,
    node: &mut Node,
    delay: u32,
    priority: TickPriority,
) {
    node.due_tick = Some(scheduler.schedule_tick(slot, delay, priority));
}
