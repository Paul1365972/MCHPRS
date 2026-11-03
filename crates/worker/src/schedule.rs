use std::time::{Duration, Instant};

const CATCH_UP_LIMIT: Duration = Duration::from_secs(2);

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TickRate {
    Paused,
    PerSecond(f64),
    Unlimited,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SendRate {
    pub per_second: f64,
}

pub struct TickSchedule {
    rate: TickRate,
    next_tick: Option<Instant>,
    behind: bool,
}

impl TickSchedule {
    pub fn new(rate: TickRate, now: Instant) -> TickSchedule {
        let mut schedule = TickSchedule {
            rate,
            next_tick: None,
            behind: false,
        };
        schedule.restart(now);
        schedule
    }

    pub fn set_rate(&mut self, rate: TickRate, now: Instant) {
        self.rate = rate;
        self.restart(now);
    }

    pub fn restart(&mut self, now: Instant) {
        self.next_tick = Some(now);
        self.behind = false;
    }

    /// Ticks that fell due while ticks ran ahead of the rate aren't owed, so the next one waits a
    /// full interval. A tick that isn't due yet keeps its time.
    pub fn skip_overdue(&mut self, now: Instant) {
        if self.next_tick.is_none_or(|next_tick| next_tick <= now) {
            self.next_tick = match self.interval() {
                Some(interval) => now.checked_add(interval),
                None => Some(now),
            };
        }
        self.behind = false;
    }

    pub fn rate(&self) -> TickRate {
        self.rate
    }

    pub fn is_behind(&self) -> bool {
        self.behind
    }

    pub fn next_tick(&self) -> Option<Instant> {
        match self.rate {
            TickRate::Paused => None,
            TickRate::PerSecond(_) if self.interval().is_none() => None,
            _ => self.next_tick,
        }
    }

    pub fn due(&mut self, now: Instant) -> u64 {
        if self.rate == TickRate::Unlimited {
            return u64::MAX;
        }
        let (Some(interval), Some(next_tick)) = (self.interval(), self.next_tick.as_mut()) else {
            return 0;
        };
        let late = now.saturating_duration_since(*next_tick);
        self.behind = late > CATCH_UP_LIMIT;
        if self.behind {
            let skipped = (late - CATCH_UP_LIMIT)
                .as_nanos()
                .div_ceil(interval.as_nanos());
            let skipped = skipped * interval.as_nanos();
            *next_tick += Duration::from_nanos(u64::try_from(skipped).unwrap_or(u64::MAX));
        }
        if now < *next_tick {
            return 0;
        }
        let due = (now - *next_tick).as_nanos() / interval.as_nanos() + 1;
        u64::try_from(due).unwrap_or(u64::MAX)
    }

    pub fn ran(&mut self, ticks: u64) {
        let Some(interval) = self.interval() else {
            return;
        };
        let ticks = u32::try_from(ticks).unwrap_or(u32::MAX);
        self.next_tick = self
            .next_tick
            .zip(interval.checked_mul(ticks))
            .and_then(|(next_tick, elapsed)| next_tick.checked_add(elapsed));
    }

    fn interval(&self) -> Option<Duration> {
        match self.rate {
            TickRate::PerSecond(rate) if rate > 0.0 => Duration::try_from_secs_f64(1.0 / rate)
                .ok()
                .map(|interval| interval.max(Duration::from_nanos(1))),
            _ => None,
        }
    }
}

pub struct SendSchedule {
    interval: Option<Duration>,
    next_send: Option<Instant>,
}

impl SendSchedule {
    pub fn new(rate: SendRate, now: Instant) -> SendSchedule {
        let interval = if rate.per_second > 0.0 {
            Duration::try_from_secs_f64(1.0 / rate.per_second).ok()
        } else {
            None
        };
        SendSchedule {
            interval,
            next_send: interval.map(|_| now),
        }
    }

    pub fn set_rate(&mut self, rate: SendRate, now: Instant) {
        *self = SendSchedule::new(rate, now);
    }

    pub fn interval(&self) -> Option<Duration> {
        self.interval
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
