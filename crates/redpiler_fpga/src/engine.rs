use crate::circuit::{Circuit, Kind, State};
use mchprs_redpiler::compile_graph::SignalStrength;
use mchprs_redpiler::{
    Engine, EngineError, EngineSnapshot, Input, NodeChanges, NodeId, NodeOutput, PendingTick,
};
use std::error::Error;
use std::mem;
use std::time::Instant;

/// The phase engine only forwards inputs to input nodes and never advances past `u64::MAX` ticks.
pub(crate) trait Simulator: Send {
    type Error: Error + Send + Sync + 'static;

    fn input(&mut self, node: usize, powered: bool) -> Result<(), Self::Error>;
    fn advance(&mut self, ticks: u64) -> Result<(), Self::Error>;
    fn states(&mut self) -> Result<Vec<State>, Self::Error>;

    fn io_states(&mut self, io_nodes: &[usize]) -> Result<Vec<State>, Self::Error> {
        let states = self.states()?;
        Ok(io_nodes.iter().map(|&node| states[node]).collect())
    }
}

#[derive(Debug, thiserror::Error)]
enum PhaseError {
    #[error("node {node}: {kind:?} does not accept {input:?}")]
    Input {
        node: usize,
        kind: Kind,
        input: Input,
    },
    #[error("phase tick counter overflow")]
    TickOverflow,
}

pub(crate) struct PhaseEngine<S> {
    simulator: S,
    kinds: Vec<Kind>,
    io_nodes: Vec<usize>,
    io_only: bool,
    input_powered: Vec<bool>,
    reported: Vec<State>,
    ticks: u64,
    ticks_per_second: f64,
}

impl<S: Simulator> PhaseEngine<S> {
    pub(crate) fn new(simulator: S, circuit: &Circuit, io_only: bool) -> PhaseEngine<S> {
        PhaseEngine {
            simulator,
            kinds: circuit.nodes().iter().map(|node| node.kind).collect(),
            io_nodes: circuit.io_nodes().to_vec(),
            io_only,
            input_powered: circuit
                .nodes()
                .iter()
                .map(|node| node.initial.is_powered())
                .collect(),
            reported: circuit.nodes().iter().map(|node| node.initial).collect(),
            ticks: 0,
            ticks_per_second: 0.0,
        }
    }

    fn report(&mut self, node: usize, state: State, changes: &mut NodeChanges) {
        let previous = mem::replace(&mut self.reported[node], state);
        if self.kinds[node] == Kind::Constant || appearance(previous) == appearance(state) {
            return;
        }
        changes.outputs.push((
            NodeId::new(node),
            NodeOutput {
                power: state.strength,
                repeater_locked: state.locked,
            },
        ));
    }
}

fn appearance(state: State) -> (bool, SignalStrength) {
    (state.locked, state.strength)
}

impl<S: Simulator> Engine for PhaseEngine<S> {
    fn run_ticks(&mut self, max_ticks: u64, deadline: Instant) -> Result<u64, EngineError> {
        let mut completed = 0;
        while completed < max_ticks {
            let started = Instant::now();
            let budget = deadline.saturating_duration_since(started).as_secs_f64();
            let chunk = ((self.ticks_per_second * budget) as u64).clamp(1, max_ticks - completed);
            let ticks = self
                .ticks
                .checked_add(chunk)
                .ok_or(PhaseError::TickOverflow)?;
            self.simulator.advance(chunk)?;
            self.ticks = ticks;
            completed += chunk;
            let elapsed = started.elapsed().as_secs_f64();
            self.ticks_per_second = if elapsed > 0.0 {
                chunk as f64 / elapsed
            } else {
                f64::MAX
            };
            if Instant::now() >= deadline {
                break;
            }
        }
        Ok(completed)
    }

    fn input(&mut self, node: NodeId, input: Input) -> Result<(), EngineError> {
        let index = node.index();
        let powered = match (input, self.kinds[index]) {
            (Input::Interact, Kind::Lever) => !self.input_powered[index],
            (Input::Interact, Kind::Button) => true,
            (Input::PressurePlate { powered }, Kind::PressurePlate) => powered,
            (input, kind) => {
                return Err(PhaseError::Input {
                    node: index,
                    kind,
                    input,
                }
                .into());
            }
        };
        self.simulator.input(index, powered)?;
        self.input_powered[index] = powered;
        Ok(())
    }

    fn take_changes(&mut self) -> Result<NodeChanges, EngineError> {
        let mut changes = NodeChanges::default();
        if self.io_only {
            let states = self.simulator.io_states(&self.io_nodes)?;
            for (offset, state) in states.into_iter().enumerate() {
                self.report(self.io_nodes[offset], state, &mut changes);
            }
        } else {
            for (node, state) in self.simulator.states()?.into_iter().enumerate() {
                self.report(node, state, &mut changes);
            }
        }
        Ok(changes)
    }

    fn snapshot(&mut self) -> Result<EngineSnapshot, EngineError> {
        let states = self.simulator.states()?;
        let mut changes = NodeChanges::default();
        let mut pending_ticks = Vec::new();
        for (node, state) in states.into_iter().enumerate() {
            if let Some(pending) = state.pending {
                pending_ticks.push(PendingTick {
                    node: NodeId::new(node),
                    delay: u32::from(pending.delay.get()),
                    priority: pending.priority,
                });
            }
            self.report(node, state, &mut changes);
        }
        Ok(EngineSnapshot {
            changes,
            pending_ticks,
        })
    }
}
