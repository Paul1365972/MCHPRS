use mchprs_save_data::plot_data::Tps;
use std::time::{Duration, Instant};

const MAX_BATCH_DURATION: Duration = Duration::from_millis(10);
const MAX_TICK_BACKLOG: Duration = Duration::from_secs(2);
pub const PLAYER_UPDATE_INTERVAL: Duration = Duration::from_millis(50);

pub fn batch_duration(world_send_rate: f32) -> Duration {
    Duration::try_from_secs_f64(1.0 / f64::from(world_send_rate))
        .unwrap_or(Duration::MAX)
        .min(MAX_BATCH_DURATION)
}

pub struct RateSchedule {
    last_update: Instant,
    backlog: f64,
}

impl RateSchedule {
    pub fn new(now: Instant) -> Self {
        Self {
            last_update: now,
            backlog: 0.0,
        }
    }

    pub fn accrue(&mut self, now: Instant, rate: f32, max_backlog: f64) -> u64 {
        let elapsed = now.duration_since(self.last_update);
        self.last_update = now;
        self.backlog = (self.backlog + elapsed.as_secs_f64() * f64::from(rate)).min(max_backlog);
        self.backlog as u64
    }

    pub fn complete(&mut self, count: u64) {
        self.backlog = (self.backlog - count as f64).max(0.0);
    }

    pub fn wait(&self, now: Instant, rate: f32) -> Duration {
        if self.backlog >= 1.0 {
            return Duration::ZERO;
        }
        Duration::try_from_secs_f64((1.0 - self.backlog) / f64::from(rate))
            .unwrap_or(Duration::MAX)
            .saturating_sub(now.duration_since(self.last_update))
    }

    pub fn ticks_due(&mut self, now: Instant, tps: Tps) -> u64 {
        match tps {
            Tps::Limited(rate) => {
                let max_backlog = f64::from(rate) * MAX_TICK_BACKLOG.as_secs_f64();
                self.accrue(now, rate, max_backlog)
            }
            Tps::Unlimited => u64::MAX,
        }
    }

    pub fn tick_wait(&self, now: Instant, tps: Tps) -> Duration {
        match tps {
            Tps::Limited(rate) => self.wait(now, rate),
            Tps::Unlimited => Duration::ZERO,
        }
    }
}
