use std::time::Duration;

const CATCH_UP_LIMIT: Duration = Duration::from_secs(2);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TickRate {
    Paused,
    Every(Duration),
    Unlimited,
}

impl TickRate {
    pub fn per_second(rate: f64) -> TickRate {
        match Duration::try_from_secs_f64(1.0 / rate) {
            Err(_) => TickRate::Paused,
            Ok(interval) if interval.is_zero() => TickRate::Unlimited,
            Ok(interval) => TickRate::Every(interval),
        }
    }
}

pub struct Pacer {
    rate: TickRate,
    next_tick: Option<Duration>,
    behind: bool,
}

impl Pacer {
    pub fn new(rate: TickRate, now: Duration) -> Pacer {
        Pacer {
            rate,
            next_tick: Some(now),
            behind: false,
        }
    }

    pub fn rate(&self) -> TickRate {
        self.rate
    }

    pub fn set_rate(&mut self, rate: TickRate, now: Duration) {
        *self = Pacer::new(rate, now);
    }

    pub fn skip_overdue(&mut self, now: Duration) {
        if self.next_tick.is_none_or(|next_tick| next_tick <= now) {
            self.next_tick = match self.rate {
                TickRate::Every(interval) => now.checked_add(interval),
                _ => Some(now),
            };
        }
        self.behind = false;
    }

    pub fn is_behind(&self) -> bool {
        self.behind
    }

    pub fn next_tick(&self) -> Option<Duration> {
        match self.rate {
            TickRate::Paused => None,
            _ => self.next_tick,
        }
    }

    pub fn due(&mut self, now: Duration) -> Option<u64> {
        let interval = match self.rate {
            TickRate::Paused => return None,
            TickRate::Unlimited => return Some(u64::MAX),
            TickRate::Every(interval) => interval,
        };
        let next_tick = self.next_tick.as_mut()?;
        let late = now.saturating_sub(*next_tick);
        self.behind = late > CATCH_UP_LIMIT;
        if self.behind {
            let skipped = (late - CATCH_UP_LIMIT)
                .as_nanos()
                .div_ceil(interval.as_nanos());
            let skipped = skipped * interval.as_nanos();
            *next_tick += Duration::from_nanos(u64::try_from(skipped).unwrap_or(u64::MAX));
        }
        if now < *next_tick {
            return None;
        }
        let due = (now - *next_tick).as_nanos() / interval.as_nanos() + 1;
        Some(u64::try_from(due).unwrap_or(u64::MAX))
    }

    pub fn ran(&mut self, ticks: u64) {
        let TickRate::Every(interval) = self.rate else {
            return;
        };
        let ticks = u32::try_from(ticks).unwrap_or(u32::MAX);
        self.next_tick = self
            .next_tick
            .zip(interval.checked_mul(ticks))
            .and_then(|(next_tick, elapsed)| next_tick.checked_add(elapsed));
    }
}
