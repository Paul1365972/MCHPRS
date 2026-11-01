use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SendRate {
    Never,
    PerSecond(f64),
}

impl SendRate {
    pub fn per_second(rate: f64) -> SendRate {
        if rate > 0.0 {
            SendRate::PerSecond(rate)
        } else {
            SendRate::Never
        }
    }
}

pub struct SendSchedule {
    rate: SendRate,
    last_send: Instant,
}

impl SendSchedule {
    pub fn new(rate: SendRate, now: Instant) -> SendSchedule {
        SendSchedule {
            rate,
            last_send: now,
        }
    }

    pub fn set_rate(&mut self, rate: SendRate) {
        self.rate = rate;
    }

    pub fn until_send(&self, now: Instant) -> Option<Duration> {
        let SendRate::PerSecond(per_second) = self.rate else {
            return None;
        };
        let interval = Duration::try_from_secs_f64(1.0 / per_second).unwrap_or(Duration::MAX);
        Some(interval.saturating_sub(now.saturating_duration_since(self.last_send)))
    }

    pub fn is_due(&self, now: Instant) -> bool {
        self.until_send(now).is_some_and(|wait| wait.is_zero())
    }

    pub fn sent(&mut self, now: Instant) {
        self.last_send = now;
    }
}
