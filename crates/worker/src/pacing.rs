use super::*;

const MAX_BACKLOG: Duration = Duration::from_secs(1);

pub(super) struct Pacing {
    pub(super) tick_rate: TickRate,
    pub(super) next_tick: Instant,
    pub(super) send_interval: Option<Duration>,
    pub(super) next_send: Instant,
}

impl Pacing {
    pub(super) fn new(tick_rate: TickRate, send_rate: SendRate) -> Pacing {
        let mut pacing = Pacing {
            tick_rate,
            next_tick: Instant::now(),
            send_interval: None,
            next_send: Instant::now(),
        };
        pacing.set_send_rate(send_rate);
        pacing
    }

    pub(super) fn forget_backlog(&mut self) {
        self.next_tick = Instant::now();
        self.next_send = Instant::now();
    }

    pub(super) fn set_tick_rate(&mut self, rate: TickRate) {
        self.tick_rate = rate;
        self.next_tick = Instant::now();
    }

    pub(super) fn set_send_rate(&mut self, rate: SendRate) {
        self.send_interval = if rate.per_second > 0.0 {
            Duration::try_from_secs_f64(1.0 / rate.per_second).ok()
        } else {
            None
        };
        self.next_send = Instant::now();
    }

    pub(super) fn tick_interval(&self) -> Option<Duration> {
        match self.tick_rate {
            TickRate::PerSecond(rate) if rate > 0.0 => Duration::try_from_secs_f64(1.0 / rate).ok(),
            _ => None,
        }
    }

    pub(super) fn due_ticks(&mut self, now: Instant) -> u64 {
        if self.tick_rate == TickRate::Unlimited {
            return u64::MAX;
        }
        let Some(interval) = self.tick_interval() else {
            return 0;
        };
        if now < self.next_tick {
            return 0;
        }
        if now - self.next_tick > MAX_BACKLOG {
            self.next_tick = now;
        }
        let interval = interval.as_nanos().max(1);
        let behind = (now - self.next_tick).as_nanos() / interval + 1;
        u64::try_from(behind).unwrap_or(u64::MAX)
    }

    pub(super) fn ran_ticks(&mut self, ticks: u64) {
        if let Some(interval) = self.tick_interval() {
            self.next_tick += interval * u32::try_from(ticks).unwrap_or(u32::MAX);
        }
    }
}
