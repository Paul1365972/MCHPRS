mod aggregate;
mod settle;
mod timers;

pub(crate) use settle::settled_initial_state;

use crate::circuit::{Circuit, Kind, PendingTick, Snapshot, State, PULSE_PRIORITY};
use crate::engine::Simulator;
use aggregate::Aggregate;
use mchprs_redpiler::compile_graph::SignalStrength;
use settle::settle_node;
use std::io;
use timers::Timers;

#[derive(Clone, Copy)]
struct Consumer {
    target: usize,
    attenuation: u8,
    side: bool,
}

pub(crate) struct Reference {
    kinds: Vec<Kind>,
    states: Vec<State>,
    consumers: Vec<Vec<Consumer>>,
    aggregates: Vec<[Aggregate; 2]>,
    timers: Timers,
    phases: [Vec<usize>; 4],
    strength_changes: Vec<(usize, SignalStrength, SignalStrength)>,
    affected: Vec<usize>,
    affected_flags: Vec<bool>,
    ticks: u64,
}

impl Reference {
    pub fn new(circuit: &Circuit) -> io::Result<Self> {
        let count = circuit.nodes().len();
        let mut engine = Self {
            kinds: circuit.nodes().iter().map(|node| node.kind).collect(),
            states: circuit.nodes().iter().map(|node| node.initial).collect(),
            consumers: vec![Vec::new(); count],
            aggregates: vec![Default::default(); count],
            timers: Timers::new(count),
            phases: Default::default(),
            strength_changes: Vec::new(),
            affected: Vec::new(),
            affected_flags: vec![false; count],
            ticks: 0,
        };
        for (index, node) in circuit.nodes().iter().enumerate() {
            engine.states[index].pending = None;
            if let Some(pending) = node.initial.pending {
                engine.schedule(index, pending);
            }
            if !node.kind.observes_inputs() {
                continue;
            }
            for (side, links) in [(false, &node.inputs), (true, &node.sides)] {
                for link in links {
                    engine.aggregates[index][usize::from(side)]
                        .insert(link.initial_strength(circuit.nodes()));
                    if link.attenuation < 15 {
                        engine.consumers[link.source].push(Consumer {
                            target: index,
                            attenuation: link.attenuation,
                            side,
                        });
                    }
                }
            }
        }
        Ok(engine)
    }

    pub fn input(&mut self, index: usize, value: bool) -> io::Result<()> {
        let kind = self.kinds.get(index).copied().ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidInput, "input index is out of bounds")
        })?;
        let mut state = self.states[index];
        match kind {
            Kind::Lever | Kind::PressurePlate => state.set_powered(value),
            Kind::Button => {
                if value && !state.is_powered() {
                    state.set_powered(true);
                    if let Some(tick) = kind.scheduled_tick(true) {
                        self.schedule(index, tick);
                    }
                }
            }
            _ => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "node is not an input kind",
                ));
            }
        }
        self.commit(index, state);
        self.settle();
        Ok(())
    }

    pub fn step(&mut self, ticks: u64) -> io::Result<()> {
        let target = self.ticks.checked_add(ticks).ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidInput, "phase tick counter overflow")
        })?;
        while let Some(deadline) = self.timers.next_deadline() {
            if deadline > u128::from(target) {
                break;
            }
            self.ticks = deadline as u64;
            self.timers.freeze(deadline, &mut self.phases);
            for phase in 0..4 {
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

    fn state(&self, index: usize) -> State {
        let mut state = self.states[index];
        if let Some(pending) = self.timers.remaining(index, self.ticks) {
            state.pending = Some(pending);
        }
        state
    }

    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            ticks: self.ticks,
            states: (0..self.states.len())
                .map(|index| self.state(index))
                .collect(),
        }
    }

    fn schedule(&mut self, index: usize, tick: PendingTick) {
        let deadline = u128::from(self.ticks) + u128::from(tick.delay.get());
        self.timers.schedule(index, deadline, tick.priority);
    }

    fn commit(&mut self, index: usize, state: State) {
        let previous = self.states[index];
        if previous.strength != state.strength {
            self.strength_changes
                .push((index, previous.strength, state.strength));
        }
        self.states[index] = state;
    }

    fn fire(&mut self, index: usize) {
        let input = self.aggregates[index][0].maximum();
        let side = self.aggregates[index][1].maximum();
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
        for (source, previous, strength) in self.strength_changes.drain(..) {
            for consumer in &self.consumers[source] {
                let previous = previous.saturating_sub(consumer.attenuation);
                let strength = strength.saturating_sub(consumer.attenuation);
                if previous == strength {
                    continue;
                }
                self.aggregates[consumer.target][usize::from(consumer.side)]
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
            let input = self.aggregates[index][0].maximum();
            let side = self.aggregates[index][1].maximum();
            let pending = self.timers.is_pending(index);
            let (state, tick) =
                settle_node(self.kinds[index], self.states[index], input, side, pending);
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
    fn input(&mut self, node: usize, powered: bool) -> io::Result<()> {
        Reference::input(self, node, powered)
    }

    fn advance(&mut self, ticks: u64) -> io::Result<()> {
        self.step(ticks)
    }

    fn states(&mut self) -> io::Result<Vec<State>> {
        Ok(self.snapshot().states)
    }
}
