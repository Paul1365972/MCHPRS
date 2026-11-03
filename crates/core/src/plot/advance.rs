use super::Plot;
use crate::player::PacketSender;
use mchprs_network::PlayerPacketSender;
use mchprs_redstone::pace::{Advanced, Goal};
use std::time::Instant;

pub(super) struct Advance {
    goal: Goal,
    ran: u64,
    requester: PlayerPacketSender,
    started: Instant,
    phase: Phase,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Waiting,
    Running,
}

impl Advance {
    fn remaining_goal(&self) -> Goal {
        match self.goal {
            Goal::Ticks(ticks) => Goal::Ticks(ticks - self.ran),
            Goal::Settled => Goal::Settled,
        }
    }

    fn has_reached(&self, settled: bool) -> bool {
        match self.goal {
            Goal::Ticks(ticks) => self.ran == ticks,
            Goal::Settled => settled,
        }
    }

    fn reply(&self, reached: bool) {
        let elapsed = self.started.elapsed();
        let ran = self.ran;
        let message = match (self.goal, reached) {
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

impl Plot {
    pub fn start_advancing(&mut self, goal: Goal, requester: PlayerPacketSender) -> bool {
        if self.advance.is_some() {
            return false;
        }
        self.advance = Some(Advance {
            goal,
            ran: 0,
            requester,
            started: Instant::now(),
            phase: Phase::Waiting,
        });
        true
    }

    pub fn stop_advancing(&mut self) -> bool {
        if self.advance.is_none() {
            return false;
        }
        self.end_world_advance();
        self.publish_world();
        self.end_advance(false);
        true
    }

    pub(super) fn continue_advancing(&mut self) {
        let Some(advance) = &mut self.advance else {
            return;
        };
        if advance.phase != Phase::Waiting {
            return;
        }
        advance.phase = Phase::Running;
        let goal = advance.remaining_goal();
        self.pacer.advance(Some(goal), self.now());
    }

    pub(super) fn end_world_advance(&mut self) {
        if let Some(advanced) = self.pacer.advance(None, self.now()) {
            self.record_advance(advanced);
        }
    }

    pub(super) fn record_advance(&mut self, advanced: Advanced) {
        let Some(advance) = &mut self.advance else {
            return;
        };
        advance.ran += advanced.ran;
        if advance.has_reached(advanced.settled) {
            self.end_advance(true);
        } else {
            advance.phase = Phase::Waiting;
        }
    }

    fn end_advance(&mut self, reached: bool) {
        if let Some(advance) = self.advance.take() {
            advance.reply(reached);
        }
    }
}
