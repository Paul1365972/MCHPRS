use mchprs_save_data::plot_data::WorldSendRate;
use std::time::{Duration, Instant};

pub struct SendSchedule {
    interval: Option<Duration>,
    next_send: Option<Instant>,
}

impl SendSchedule {
    pub fn new(rate: WorldSendRate, now: Instant) -> SendSchedule {
        let interval = if rate.0 > 0.0 {
            Duration::try_from_secs_f64(1.0 / f64::from(rate.0)).ok()
        } else {
            None
        };
        SendSchedule {
            interval,
            next_send: interval.map(|_| now),
        }
    }

    pub fn set_rate(&mut self, rate: WorldSendRate, now: Instant) {
        *self = SendSchedule::new(rate, now);
    }

    pub fn next_send(&self) -> Option<Instant> {
        self.next_send
    }

    pub fn is_due(&self, now: Instant) -> bool {
        self.next_send.is_some_and(|next_send| now >= next_send)
    }

    pub fn sent(&mut self, now: Instant) {
        self.next_send = self.interval.and_then(|interval| now.checked_add(interval));
    }
}
