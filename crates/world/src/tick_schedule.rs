use std::time::{Duration, Instant};

const BACKLOG_LIMIT: Duration = Duration::from_secs(2);

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TickRate {
    Paused,
    PerSecond(f64),
    Unlimited,
    Ticks(u64),
}

impl TickRate {
    pub fn per_second(rate: f64) -> TickRate {
        if rate > 0.0 {
            TickRate::PerSecond(rate)
        } else {
            TickRate::Paused
        }
    }

    pub fn ticks_owed(self) -> u64 {
        match self {
            TickRate::Ticks(ticks) => ticks,
            _ => 0,
        }
    }

    pub fn replaces(self, current: TickRate) -> bool {
        matches!(self, TickRate::Ticks(_)) || self != current
    }
}

pub enum Step {
    Run(u64),
    WaitFor(Duration),
    WaitForRequest,
}

pub struct TickSchedule {
    rate: TickRate,
    backlog: f64,
    last_update: Instant,
    saturated: bool,
}

impl TickSchedule {
    pub fn new(now: Instant) -> TickSchedule {
        TickSchedule {
            rate: TickRate::Paused,
            backlog: 0.0,
            last_update: now,
            saturated: false,
        }
    }

    pub fn set_rate(&mut self, rate: TickRate, now: Instant) {
        if !rate.replaces(self.rate) {
            return;
        }
        self.rate = rate;
        self.restart(now);
    }

    pub fn restart(&mut self, now: Instant) {
        self.backlog = 0.0;
        self.last_update = now;
        self.saturated = false;
    }

    pub fn is_saturated(&self) -> bool {
        self.saturated
    }

    pub fn ticks_owed(&self) -> u64 {
        self.rate.ticks_owed()
    }

    pub fn next(&mut self, now: Instant) -> Step {
        match self.rate {
            TickRate::Paused | TickRate::Ticks(0) => Step::WaitForRequest,
            TickRate::Ticks(owed) => Step::Run(owed),
            TickRate::Unlimited => Step::Run(u64::MAX),
            TickRate::PerSecond(per_second) => {
                let elapsed = now.saturating_duration_since(self.last_update);
                self.last_update = now;
                let backlog_limit = (BACKLOG_LIMIT.as_secs_f64() * per_second).max(1.0);
                let accrued = self.backlog + elapsed.as_secs_f64() * per_second;
                self.saturated = accrued - backlog_limit >= 1.0;
                self.backlog = accrued.min(backlog_limit);
                let due = self.backlog as u64;
                if due != 0 {
                    Step::Run(due)
                } else {
                    Step::WaitFor(
                        Duration::try_from_secs_f64((1.0 - self.backlog) / per_second)
                            .unwrap_or(Duration::MAX),
                    )
                }
            }
        }
    }

    pub fn complete(&mut self, ticks: u64) {
        match &mut self.rate {
            TickRate::PerSecond(_) => self.backlog -= ticks as f64,
            TickRate::Ticks(owed) => *owed = owed.saturating_sub(ticks),
            TickRate::Paused | TickRate::Unlimited => {}
        }
    }
}
