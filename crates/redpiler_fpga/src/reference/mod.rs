mod aggregate;
mod settle;
mod timers;

pub(crate) use settle::settled_initial_state;

use crate::circuit::{Circuit, Kind, PendingTick, State, PULSE_PRIORITY};
use crate::engine::Simulator;
use aggregate::Aggregate;
use mchprs_redpiler::compile_graph::SignalStrength;
use settle::settle_node;
use std::convert::Infallible;
use timers::Timers;

#[derive(Clone, Copy)]
enum LinkSide {
    Rear,
    Side,
}

#[derive(Clone, Default)]
struct Inputs {
    rear: Aggregate,
    side: Aggregate,
}

impl Inputs {
    fn get_mut(&mut self, side: LinkSide) -> &mut Aggregate {
        match side {
            LinkSide::Rear => &mut self.rear,
            LinkSide::Side => &mut self.side,
        }
    }
}

#[derive(Clone, Copy)]
struct Consumer {
    target: usize,
    attenuation: u8,
    side: LinkSide,
}

struct StrengthChange {
    node: usize,
    previous: SignalStrength,
    strength: SignalStrength,
}

pub(crate) struct Reference {
    kinds: Vec<Kind>,
    states: Vec<State>,
    consumers: Vec<Vec<Consumer>>,
    inputs: Vec<Inputs>,
    timers: Timers,
    phases: [Vec<usize>; 4],
    strength_changes: Vec<StrengthChange>,
    affected: Vec<usize>,
    affected_flags: Vec<bool>,
    ticks: u64,
}

impl Reference {
    pub(crate) fn new(circuit: &Circuit) -> Self {
        let count = circuit.nodes().len();
        let mut reference = Self {
            kinds: circuit.nodes().iter().map(|node| node.kind).collect(),
            states: circuit
                .nodes()
                .iter()
                .map(|node| State {
                    pending: None,
                    ..node.initial
                })
                .collect(),
            consumers: vec![Vec::new(); count],
            inputs: vec![Inputs::default(); count],
            timers: Timers::new(count),
            phases: Default::default(),
            strength_changes: Vec::new(),
            affected: Vec::new(),
            affected_flags: vec![false; count],
            ticks: 0,
        };
        for (index, node) in circuit.nodes().iter().enumerate() {
            if let Some(pending) = node.initial.pending {
                reference.schedule(index, pending);
            }
            if !node.kind.observes_inputs() {
                continue;
            }
            for (side, links) in [
                (LinkSide::Rear, &node.inputs),
                (LinkSide::Side, &node.sides),
            ] {
                for link in links {
                    reference.inputs[index]
                        .get_mut(side)
                        .insert(link.initial_strength(circuit.nodes()));
                    if link.attenuation < 15 {
                        reference.consumers[link.source].push(Consumer {
                            target: index,
                            attenuation: link.attenuation,
                            side,
                        });
                    }
                }
            }
        }
        reference
    }

    fn state(&self, index: usize) -> State {
        State {
            pending: self.timers.remaining(index, self.ticks),
            ..self.states[index]
        }
    }

    fn schedule(&mut self, index: usize, tick: PendingTick) {
        let deadline = u128::from(self.ticks) + u128::from(tick.delay.get());
        self.timers.schedule(index, deadline, tick.priority);
    }

    fn commit(&mut self, index: usize, state: State) {
        let previous = self.states[index];
        if previous.strength != state.strength {
            self.strength_changes.push(StrengthChange {
                node: index,
                previous: previous.strength,
                strength: state.strength,
            });
        }
        self.states[index] = state;
    }

    fn fire(&mut self, index: usize) {
        let input = self.inputs[index].rear.maximum();
        let side = self.inputs[index].side.maximum();
        let mut state = self.states[index];
        let kind = self.kinds[index];
        match kind {
            Kind::Repeater { .. } if !state.locked => {
                if state.is_powered() && input.is_zero() {
                    state.set_powered(false);
                } else if !state.is_powered() {
                    state.set_powered(true);
                    if input.is_zero()
                        && let Some(delay) = kind.delay()
                    {
                        let pulse = PendingTick {
                            delay,
                            priority: PULSE_PRIORITY,
                        };
                        self.schedule(index, pulse);
                    }
                }
            }
            Kind::Torch => state.set_powered(input.is_zero()),
            Kind::Comparator {
                mode, far_input, ..
            } => state.strength = mode.output(far_input, input, side),
            Kind::Lamp if input.is_zero() => state.set_powered(false),
            Kind::Button => state.set_powered(false),
            _ => {}
        }
        self.commit(index, state);
    }

    fn settle(&mut self) {
        for change in self.strength_changes.drain(..) {
            for consumer in &self.consumers[change.node] {
                let previous = change.previous.saturating_sub(consumer.attenuation);
                let strength = change.strength.saturating_sub(consumer.attenuation);
                if previous == strength {
                    continue;
                }
                self.inputs[consumer.target]
                    .get_mut(consumer.side)
                    .replace(previous, strength);
                if !self.affected_flags[consumer.target] {
                    self.affected_flags[consumer.target] = true;
                    self.affected.push(consumer.target);
                }
            }
        }
        for offset in 0..self.affected.len() {
            let index = self.affected[offset];
            self.affected_flags[index] = false;
            let inputs = &self.inputs[index];
            let pending = self.timers.is_pending(index);
            let (state, tick) = settle_node(
                self.kinds[index],
                self.states[index],
                inputs.rear.maximum(),
                inputs.side.maximum(),
                pending,
            );
            if let Some(tick) = tick {
                self.schedule(index, tick);
            }
            self.commit(index, state);
        }
        self.affected.clear();
        self.strength_changes.clear();
    }
}

impl Simulator for Reference {
    type Error = Infallible;

    fn input(&mut self, node: usize, powered: bool) -> Result<(), Infallible> {
        let kind = self.kinds[node];
        let mut state = self.states[node];
        if kind == Kind::Button {
            if powered
                && !state.is_powered()
                && let Some(tick) = kind.scheduled_tick(true)
            {
                state.set_powered(true);
                self.schedule(node, tick);
            }
        } else {
            state.set_powered(powered);
        }
        self.commit(node, state);
        self.settle();
        Ok(())
    }

    fn advance(&mut self, ticks: u64) -> Result<(), Infallible> {
        let target = self.ticks + ticks;
        while let Some(deadline) = self.timers.next_deadline() {
            if deadline > u128::from(target) {
                break;
            }
            self.ticks = deadline as u64;
            self.timers.freeze(deadline, &mut self.phases);
            for phase in 0..self.phases.len() {
                for offset in 0..self.phases[phase].len() {
                    let index = self.phases[phase][offset];
                    self.timers.consume(index);
                    self.fire(index);
                }
                self.phases[phase].clear();
                self.settle();
            }
        }
        self.ticks = target;
        Ok(())
    }

    fn states(&mut self) -> Result<Vec<State>, Infallible> {
        Ok((0..self.states.len())
            .map(|index| self.state(index))
            .collect())
    }
}
