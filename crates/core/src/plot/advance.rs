use super::Plot;
use crate::player::PacketSender;
use mchprs_network::PlayerPacketSender;
use mchprs_save_data::plot_data::Tps;
use mchprs_simulation::Run;
use std::collections::VecDeque;
use std::time::Instant;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Goal {
    Ticks(u64),
    Settled,
}

impl Goal {
    fn run(self) -> Run {
        match self {
            Goal::Ticks(ticks) => Run::Ticks(ticks),
            Goal::Settled => Run::UntilSettled,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Progress {
    pub goal: Goal,
    pub ran: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum AdvanceError {
    #[error("The plot is already advancing. Use /radv stop first.")]
    Advancing,
    #[error("The plot can't advance while Redpiler compiles.")]
    Compiling,
    #[error("The {0} backend can't tell when the plot has settled.")]
    Unsettling(String),
}

struct AdvanceRequest {
    goal: Goal,
    requester: PlayerPacketSender,
    started: Instant,
    start_tick: u64,
    stopped: bool,
}

impl AdvanceRequest {
    fn reply(&self, tick: u64, finished: bool) {
        let ran = tick.saturating_sub(self.start_tick);
        let elapsed = self.started.elapsed();
        let message = match (self.goal, finished) {
            (Goal::Ticks(ticks), true) => {
                format!("Plot has been advanced by {ticks} ticks ({elapsed:?})")
            }
            (Goal::Settled, true) => format!("The plot settled after {ran} ticks ({elapsed:?})"),
            (Goal::Ticks(ticks), false) => {
                format!("Stopped advancing the plot after {ran} of {ticks} ticks.")
            }
            (Goal::Settled, false) => {
                format!("Stopped advancing the plot after {ran} ticks, before it settled.")
            }
        };
        self.requester.send_system_message(&message);
    }
}

#[derive(Default)]
pub(super) struct Advances {
    sent: VecDeque<AdvanceRequest>,
}

impl Advances {
    pub(super) fn is_running(&self) -> bool {
        self.sent.iter().any(|request| !request.stopped)
    }

    pub(super) fn progress(&self, tick: u64) -> Option<Progress> {
        let request = self.sent.iter().find(|request| !request.stopped)?;
        Some(Progress {
            goal: request.goal,
            ran: tick.saturating_sub(request.start_tick),
        })
    }

    pub(super) fn end(&mut self, tick: u64, finished: bool) {
        if let Some(request) = self.sent.pop_front() {
            request.reply(tick, finished);
        }
    }

    pub(super) fn abandon(&mut self, tick: u64) {
        for request in self.sent.drain(..) {
            request.reply(tick, false);
        }
    }
}

impl Plot {
    pub fn start_advancing(
        &mut self,
        goal: Goal,
        requester: PlayerPacketSender,
    ) -> Result<(), AdvanceError> {
        if self.advances.is_running() {
            return Err(AdvanceError::Advancing);
        }
        if self.simulation.is_compiling() {
            return Err(AdvanceError::Compiling);
        }
        if goal == Goal::Settled
            && let Some(backend) = self.simulation.unsettling_backend()
        {
            return Err(AdvanceError::Unsettling(backend.to_owned()));
        }
        self.advances.sent.push_back(AdvanceRequest {
            goal,
            requester,
            started: Instant::now(),
            start_tick: self.ticks,
            stopped: false,
        });
        self.set_paused();
        self.send_run(Some(goal.run()));
        self.publish_world();
        Ok(())
    }

    pub fn stop_advancing(&mut self) -> bool {
        if !self.advances.is_running() {
            return false;
        }
        for request in &mut self.advances.sent {
            request.stopped = true;
        }
        self.send_run(None);
        self.publish_world();
        true
    }

    pub(super) fn set_paused(&mut self) {
        self.tps = Tps::Limited(0.0);
        self.world.mark_unsaved();
    }
}
