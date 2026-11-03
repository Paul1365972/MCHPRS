use super::{Plot, ADVANCE_BATCH_TIME};
use crate::player::PacketSender;
use mchprs_network::PlayerPacketSender;
use mchprs_save_data::plot_data::Tps;
use mchprs_worker::Goal;
use std::collections::VecDeque;
use std::time::{Duration, Instant};

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
    on_worker: bool,
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

    pub fn is_on_worker(&self) -> bool {
        self.on_worker
    }

    pub fn next(&self) -> Option<Goal> {
        if self.on_worker {
            return None;
        }
        self.queue.front().map(Advance::rest)
    }

    pub fn hand_to_worker(&mut self) -> Option<Goal> {
        let goal = self.next()?;
        self.on_worker = true;
        Some(goal)
    }

    pub fn ran(&mut self, ticks: u64, settled: bool) {
        if let Some(advance) = self.queue.front_mut() {
            advance.ran += ticks;
            if advance.has_reached(settled) {
                advance.reply(true);
                self.queue.pop_front();
            }
        }
        self.end_stopped();
    }

    pub fn worker_ran(&mut self, ticks: u64, settled: bool) {
        self.on_worker = false;
        self.ran(ticks, settled);
    }

    pub fn worker_ended(&mut self) {
        self.on_worker = false;
        self.end_stopped();
    }

    pub fn stop(&mut self) {
        for advance in &mut self.queue {
            advance.stopped = true;
        }
        self.end_stopped();
    }

    fn end_stopped(&mut self) {
        if self.on_worker {
            return;
        }
        while let Some(advance) = self.queue.front()
            && advance.stopped
        {
            advance.reply(false);
            self.queue.pop_front();
        }
    }

    pub fn progress(&self, worker_ran: u64) -> Option<Progress> {
        let advance = self.queue.front()?;
        Some(Progress {
            goal: advance.goal,
            ran: advance.ran + worker_ran,
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
            Some(redpiler) if self.advances.is_on_worker() => redpiler.worker.stop_advancing(),
            _ => self.publish_world(),
        }
        self.advances.stop();
        true
    }

    pub(super) fn continue_advancing(&mut self) {
        if let Some(redpiler) = &self.redpiler {
            if let Some(goal) = self.advances.hand_to_worker() {
                redpiler.worker.advance(goal);
                self.publish_world();
            }
            return;
        }
        let Some(goal) = self.advances.next() else {
            return;
        };
        let (limit, until_settled) = match goal {
            Goal::Ticks(ticks) => (ticks, false),
            Goal::Settled => (u64::MAX, true),
        };
        let start = Instant::now();
        let mut ran = 0;
        let mut settled = false;
        while ran < limit && start.elapsed() < ADVANCE_BATCH_TIME {
            if until_settled && self.world.ticks.entries().is_empty() {
                settled = true;
                break;
            }
            self.tick();
            ran += 1;
        }
        if ran > 0 {
            self.tick_time = Some(start.elapsed().div_f64(ran as f64));
            self.skip_overdue_ticks();
        }
        if settled || ran == limit {
            self.publish_world();
        }
        self.advances.ran(ran, settled);
    }

    /// Ticks that fell due while the plot advanced aren't owed, so the next one waits a full
    /// interval. A tick that isn't due yet keeps its time.
    fn skip_overdue_ticks(&mut self) {
        let now = Instant::now();
        let interval = match self.tps {
            Tps::Limited(tps) if tps > 0.0 => {
                Duration::try_from_secs_f64(1.0 / f64::from(tps)).unwrap_or(Duration::MAX)
            }
            _ => Duration::ZERO,
        };
        let lag_time = self.lag_time + (now - self.last_update_time);
        self.lag_time = if lag_time < interval {
            lag_time
        } else {
            Duration::ZERO
        };
        self.last_update_time = now;
    }
}
