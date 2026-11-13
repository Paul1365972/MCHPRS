use super::{play, Step};
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::graph::NodeState;
use mchprs_redpiler::nodes::{Changes, Nodes, Snapshot};
use mchprs_redstone::tick_queue::PendingTick;
use mchprs_simulation::readback::Readback;
use mchprs_simulation::ticks::SimulationError;
use mchprs_simulation::Simulation;
use std::slice;

#[derive(Debug, Clone)]
pub struct Seen {
    pub steps_played: usize,
    pub view: View,
}

/// `nodes` has one entry per observed node, in the order of `observed`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct View {
    pub nodes: Vec<NodeView>,
    pub pending_ticks: Option<PendingTicks>,
}

impl View {
    pub(super) fn sight(&self) -> Sight {
        match self.pending_ticks {
            None => Sight::Nodes,
            Some(_) => Sight::NodesAndPendingTicks,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PendingTicks(Vec<(NodeId, PendingTick)>);

impl PendingTicks {
    /// `pending_ticks` has the ticks of one delay and priority in the order they run in
    pub fn new(mut pending_ticks: Vec<(NodeId, PendingTick)>) -> PendingTicks {
        pending_ticks.sort_by_key(|&(_, tick)| tick);
        PendingTicks(pending_ticks)
    }

    pub fn iter(&self) -> slice::Iter<'_, (NodeId, PendingTick)> {
        self.0.iter()
    }
}

/// The shown state of a node and, after a tick, whether it played its note since the view after
/// the tick before
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NodeView {
    pub state: NodeState,
    pub played_note: bool,
}

/// Only a trace of the oracle's own circuit can hold the pending ticks
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sight {
    Nodes,
    NodesAndPendingTicks,
}

#[derive(Clone, PartialEq, Eq, Hash)]
pub(super) struct PlayedNotes(Vec<bool>);

impl PlayedNotes {
    pub(super) fn new(circuit: &Circuit) -> PlayedNotes {
        PlayedNotes(vec![false; circuit.len()])
    }

    pub(super) fn record(&mut self, changes: &Changes) {
        for &node in &changes.played_note_blocks {
            self.0[node.index()] = true;
        }
    }

    pub(super) fn take_view(
        &mut self,
        snapshot: Snapshot,
        observed: &[NodeId],
        sight: Sight,
        tick_ended: bool,
    ) -> View {
        let nodes = observed
            .iter()
            .map(|node| NodeView {
                state: snapshot.states[node.index()].shown(),
                played_note: tick_ended && self.0[node.index()],
            })
            .collect();
        if tick_ended {
            self.0.fill(false);
        }
        View {
            nodes,
            pending_ticks: match sight {
                Sight::Nodes => None,
                Sight::NodesAndPendingTicks => Some(PendingTicks::new(snapshot.pending_ticks)),
            },
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum TraceError {
    #[error(transparent)]
    Simulation(#[from] SimulationError),
    #[error("the engine's readback is incomplete, and a trace holds what every node shows")]
    IncompleteReadback,
}

pub struct Tracer<'a> {
    observed: &'a [NodeId],
    sight: Sight,
    played_notes: PlayedNotes,
    trace: Vec<Seen>,
}

impl<'a> Tracer<'a> {
    pub fn new(
        simulation: &mut dyn Simulation<Nodes>,
        circuit: &Circuit,
        observed: &'a [NodeId],
        sight: Sight,
    ) -> Result<Tracer<'a>, TraceError> {
        let mut tracer = Tracer {
            observed,
            sight,
            played_notes: PlayedNotes::new(circuit),
            trace: Vec::new(),
        };
        tracer.see(simulation, false)?;
        Ok(tracer)
    }

    pub fn play(
        &mut self,
        simulation: &mut dyn Simulation<Nodes>,
        step: Step,
    ) -> Result<Changes, TraceError> {
        play(simulation, step)?;
        let mut changes = Changes::default();
        simulation.show(mchprs_simulation::View::Observed, &mut changes)?;
        self.played_notes.record(&changes);
        self.see(simulation, matches!(step, Step::Tick))?;
        Ok(changes)
    }

    fn see(
        &mut self,
        simulation: &mut dyn Simulation<Nodes>,
        tick_ended: bool,
    ) -> Result<(), TraceError> {
        let Readback::Complete(snapshot) = simulation.read()? else {
            return Err(TraceError::IncompleteReadback);
        };
        let view = self
            .played_notes
            .take_view(snapshot, self.observed, self.sight, tick_ended);
        let steps_played = self.trace.len();
        self.trace.push(Seen { steps_played, view });
        Ok(())
    }

    pub fn trace(&self) -> &[Seen] {
        &self.trace
    }
}

pub fn trace(
    simulation: &mut dyn Simulation<Nodes>,
    circuit: &Circuit,
    observed: &[NodeId],
    sight: Sight,
    steps: &[Step],
) -> Result<Vec<Seen>, TraceError> {
    let mut tracer = Tracer::new(simulation, circuit, observed, sight)?;
    for &step in steps {
        tracer.play(simulation, step)?;
    }
    Ok(tracer.trace)
}
