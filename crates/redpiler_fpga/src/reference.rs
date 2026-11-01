mod aggregate;
mod timers;

use self::aggregate::Aggregate;
use self::timers::Timers;
use crate::circuit::{Circuit, ComparatorMode, Kind, Link, Snapshot, State};
use crate::engine::Simulator;
use std::io;

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
    strength_changes: Vec<(usize, u8, u8)>,
    affected: Vec<usize>,
    affected_flags: Vec<bool>,
    ticks: u64,
}

fn comparator(mode: ComparatorMode, far_input: Option<u8>, input: u8, side: u8) -> u8 {
    let rear = if input < 15 {
        far_input.unwrap_or(input)
    } else {
        input
    };
    match mode {
        ComparatorMode::Compare if rear >= side => rear,
        ComparatorMode::Compare => 0,
        ComparatorMode::Subtract => rear.saturating_sub(side),
    }
}

fn powered(state: &mut State, value: bool) {
    state.powered = value;
    state.strength = if value { 15 } else { 0 };
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Schedule {
    delay: u8,
    priority: u8,
}

fn settle_node(
    kind: Kind,
    mut state: State,
    input: u8,
    side: u8,
    pending: bool,
) -> (State, Option<Schedule>) {
    let schedule = |delay, priority| Some(Schedule { delay, priority });
    let schedule = match kind {
        Kind::Wire => {
            state.strength = input;
            None
        }
        Kind::Repeater {
            delay,
            facing_diode,
        } => {
            state.locked = side != 0;
            if !state.locked && !pending && state.powered != (input != 0) {
                let priority = if facing_diode {
                    0
                } else if input == 0 {
                    1
                } else {
                    2
                };
                schedule(delay, priority)
            } else {
                None
            }
        }
        Kind::Torch if !pending && state.powered != (input == 0) => schedule(1, 3),
        Kind::Comparator {
            mode,
            far_input,
            facing_diode,
        } if !pending && comparator(mode, far_input, input, side) != state.strength => {
            schedule(1, if facing_diode { 2 } else { 3 })
        }
        Kind::Lamp if state.powered && input == 0 && !pending => schedule(2, 3),
        Kind::Lamp if !state.powered && input != 0 => {
            state.powered = true;
            None
        }
        Kind::Trapdoor if state.powered != (input != 0) => {
            state.powered = input != 0;
            state.open = state.powered;
            None
        }
        _ => None,
    };
    (state, schedule)
}

/// Settling a node whose inputs did not change leaves a settled node unchanged, and firing leaves
/// every node settled, so a node that starts settled can be settled on every clock instead of only
/// when an input changes.
pub(crate) fn is_initially_settled(circuit: &Circuit, index: usize) -> bool {
    let nodes = circuit.nodes();
    let node = &nodes[index];
    let strongest = |links: &[Link]| {
        links
            .iter()
            .map(|link| {
                nodes[link.source]
                    .initial
                    .strength
                    .saturating_sub(link.attenuation)
            })
            .max()
            .unwrap_or(0)
    };
    let pending = node.kind.is_timed() && node.initial.delay != 0;
    let settled = settle_node(
        node.kind,
        node.initial,
        strongest(&node.inputs),
        strongest(&node.sides),
        pending,
    );
    settled == (node.initial, None)
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
            if node.kind.is_timed() {
                engine.states[index].delay = 0;
                engine.states[index].priority = 0;
                if node.initial.delay != 0 {
                    engine.schedule(index, node.initial.delay, node.initial.priority);
                }
            } else if matches!(
                node.kind,
                Kind::Lever | Kind::PressurePlate | Kind::Trapdoor
            ) {
                engine.states[index].delay = 0;
                engine.states[index].priority = 0;
            }
            if !node.kind.observes_inputs() {
                continue;
            }
            for (side, links) in [(false, &node.inputs), (true, &node.sides)] {
                for link in links {
                    let strength = engine.states[link.source]
                        .strength
                        .saturating_sub(link.attenuation);
                    engine.aggregates[index][usize::from(side)].insert(strength);
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
            Kind::Lever | Kind::PressurePlate => powered(&mut state, value),
            Kind::Button => {
                if value && !state.powered {
                    powered(&mut state, true);
                    self.schedule(index, 10, 3);
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
        if let Some((delay, priority)) = self.timers.remaining(index, self.ticks) {
            state.delay = delay;
            state.priority = priority;
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

    fn schedule(&mut self, index: usize, delay: u8, priority: u8) {
        let deadline = u128::from(self.ticks) + u128::from(delay);
        self.timers.schedule(index, deadline, priority);
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
        match self.kinds[index] {
            Kind::Repeater { delay, .. } if !state.locked => {
                if state.powered && input == 0 {
                    powered(&mut state, false);
                } else if !state.powered {
                    powered(&mut state, true);
                    if input == 0 {
                        self.schedule(index, delay, 1);
                    }
                }
            }
            Kind::Torch if state.powered != (input == 0) => powered(&mut state, input == 0),
            Kind::Comparator {
                mode, far_input, ..
            } => {
                let strength = comparator(mode, far_input, input, side);
                if strength != state.strength {
                    state.strength = strength;
                    state.powered = strength != 0;
                }
            }
            Kind::Lamp if state.powered && input == 0 => powered(&mut state, false),
            Kind::Button if state.powered => powered(&mut state, false),
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
            let (state, schedule) =
                settle_node(self.kinds[index], self.states[index], input, side, pending);
            if let Some(Schedule { delay, priority }) = schedule {
                self.schedule(index, delay, priority);
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
