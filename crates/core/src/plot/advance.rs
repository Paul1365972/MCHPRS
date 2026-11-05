use super::Plot;
use crate::player::PacketSender;
use mchprs_network::PlayerPacketSender;
use mchprs_redstone::pace::{Advanced, Goal};
use std::collections::VecDeque;
use std::time::Instant;

struct Advance {
    goal: Goal,
    ran: u64,
    requester: PlayerPacketSender,
    started: Instant,
    stopped: bool,
}

impl Advance {
    fn rest(&self) -> Goal {
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Progress {
    pub goal: Goal,
    pub ran: u64,
    pub waiting: usize,
}

#[derive(Default)]
pub struct Advances {
    queue: VecDeque<Advance>,
    handed: bool,
}

impl Advances {
    pub fn push(&mut self, goal: Goal, requester: PlayerPacketSender) {
        self.queue.push_back(Advance {
            goal,
            ran: 0,
            requester,
            started: Instant::now(),
            stopped: false,
        });
    }

    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }

    pub fn is_handed(&self) -> bool {
        self.handed
    }

    pub fn hand(&mut self) -> Option<Goal> {
        if self.handed {
            return None;
        }
        let goal = self.queue.front().map(Advance::rest)?;
        self.handed = true;
        Some(goal)
    }

    pub fn ran(&mut self, advanced: Advanced) {
        self.handed = false;
        if let Some(advance) = self.queue.front_mut() {
            advance.ran += advanced.ran;
            if advance.has_reached(advanced.settled) {
                advance.reply(true);
                self.queue.pop_front();
            }
        }
        self.end_stopped();
    }

    pub fn release(&mut self) {
        self.handed = false;
        self.end_stopped();
    }

    pub fn stop(&mut self) {
        for advance in &mut self.queue {
            advance.stopped = true;
        }
        self.end_stopped();
    }

    fn end_stopped(&mut self) {
        if self.handed {
            return;
        }
        while let Some(advance) = self.queue.front()
            && advance.stopped
        {
            advance.reply(false);
            self.queue.pop_front();
        }
    }

    pub fn progress(&self, advancing: u64) -> Option<Progress> {
        let advance = self.queue.front()?;
        Some(Progress {
            goal: advance.goal,
            ran: advance.ran + advancing,
            waiting: self.queue.len() - 1,
        })
    }
}

impl Plot {
    pub fn queue_advance(&mut self, goal: Goal, requester: PlayerPacketSender) {
        self.advances.push(goal, requester);
    }

    pub fn stop_advancing(&mut self) -> bool {
        if self.advances.is_empty() {
            return false;
        }
        match &self.redpiler {
            Some(redpiler) if self.advances.is_handed() => redpiler.worker.stop_advancing(),
            _ => {
                self.end_world_advance();
                self.publish_world();
            }
        }
        self.advances.stop();
        true
    }

    pub(super) fn continue_advancing(&mut self) {
        let Some(goal) = self.advances.hand() else {
            return;
        };
        match &self.redpiler {
            Some(redpiler) => {
                redpiler.worker.advance(goal);
                self.publish_world();
            }
            None => {
                self.pacer.advance(Some(goal), self.now());
            }
        }
    }

    pub(super) fn end_world_advance(&mut self) {
        if let Some(advanced) = self.pacer.advance(None, self.now()) {
            self.advances.ran(advanced);
        }
    }
}
