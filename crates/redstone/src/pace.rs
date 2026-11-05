use std::time::Duration;

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Goal {
    Ticks(u64),
    Settled,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Advanced {
    pub ran: u64,
    pub settled: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Due {
    pub limit: u64,
    pub until_settled: bool,
}

struct Advance {
    goal: Goal,
    ran: u64,
}

pub struct Pacer {
    schedule: TickSchedule,
    advance: Option<Advance>,
}

impl Pacer {
    pub fn new(rate: TickRate, now: Duration) -> Pacer {
        Pacer {
            schedule: TickSchedule::new(rate, now),
            advance: None,
        }
    }

    pub fn rate(&self) -> TickRate {
        self.schedule.rate
    }

    pub fn set_rate(&mut self, rate: TickRate, now: Duration) {
        self.schedule.set_rate(rate, now);
    }

    pub fn skip_overdue(&mut self, now: Duration) {
        self.schedule.skip_overdue(now);
    }

    pub fn is_behind(&self) -> bool {
        self.advance.is_none() && self.schedule.behind
    }

    pub fn is_advancing(&self) -> bool {
        self.advance.is_some()
    }

    pub fn advancing(&self) -> u64 {
        self.advance.as_ref().map_or(0, |advance| advance.ran)
    }

    pub fn next_tick(&self) -> Option<Duration> {
        self.schedule.next_tick()
    }

    /// Ends the running advance, if any, and starts `goal`.
    pub fn advance(&mut self, goal: Option<Goal>, now: Duration) -> Option<Advanced> {
        let ended = self.end_advance(false, now);
        self.advance = goal.map(|goal| Advance { goal, ran: 0 });
        ended
    }

    pub fn due(&mut self, now: Duration) -> Option<Due> {
        if let Some(advance) = &self.advance {
            return Some(match advance.goal {
                Goal::Ticks(ticks) => Due {
                    limit: ticks - advance.ran,
                    until_settled: false,
                },
                Goal::Settled => Due {
                    limit: u64::MAX,
                    until_settled: true,
                },
            });
        }
        match self.schedule.due(now) {
            0 => None,
            limit => Some(Due {
                limit,
                until_settled: false,
            }),
        }
    }

    /// Ticks due during an advance aren't owed afterwards.
    pub fn ran(&mut self, ticks: u64, settled: bool, now: Duration) -> Option<Advanced> {
        let Some(advance) = &mut self.advance else {
            self.schedule.ran(ticks);
            return None;
        };
        advance.ran += ticks;
        let reached = match advance.goal {
            Goal::Ticks(total) => advance.ran == total,
            Goal::Settled => settled,
        };
        if !reached {
            return None;
        }
        self.end_advance(settled, now)
    }

    fn end_advance(&mut self, settled: bool, now: Duration) -> Option<Advanced> {
        let advance = self.advance.take()?;
        if advance.ran > 0 {
            self.schedule.skip_overdue(now);
        }
        Some(Advanced {
            ran: advance.ran,
            settled,
        })
    }
}

struct TickSchedule {
    rate: TickRate,
    next_tick: Option<Duration>,
    behind: bool,
}

impl TickSchedule {
    fn new(rate: TickRate, now: Duration) -> TickSchedule {
        TickSchedule {
            rate,
            next_tick: Some(now),
            behind: false,
        }
    }

    fn set_rate(&mut self, rate: TickRate, now: Duration) {
        *self = TickSchedule::new(rate, now);
    }

    fn skip_overdue(&mut self, now: Duration) {
        if self.next_tick.is_none_or(|next_tick| next_tick <= now) {
            self.next_tick = match self.interval() {
                Some(interval) => now.checked_add(interval),
                None => Some(now),
            };
        }
        self.behind = false;
    }

    fn next_tick(&self) -> Option<Duration> {
        match self.rate {
            TickRate::Paused => None,
            TickRate::PerSecond(_) if self.interval().is_none() => None,
            _ => self.next_tick,
        }
    }

    fn due(&mut self, now: Duration) -> u64 {
        if self.rate == TickRate::Unlimited {
            return u64::MAX;
        }
        let (Some(interval), Some(next_tick)) = (self.interval(), self.next_tick.as_mut()) else {
            return 0;
        };
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
            return 0;
        }
        let due = (now - *next_tick).as_nanos() / interval.as_nanos() + 1;
        u64::try_from(due).unwrap_or(u64::MAX)
    }

    fn ran(&mut self, ticks: u64) {
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
    next_send: Option<Duration>,
}

impl SendSchedule {
    pub fn new(rate: SendRate, now: Duration) -> SendSchedule {
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

    pub fn set_rate(&mut self, rate: SendRate, now: Duration) {
        *self = SendSchedule::new(rate, now);
    }

    pub fn interval(&self) -> Option<Duration> {
        self.interval
    }

    pub fn next_send(&self) -> Option<Duration> {
        self.next_send
    }

    pub fn is_due(&self, now: Duration) -> bool {
        self.next_send.is_some_and(|next_send| now >= next_send)
    }

    pub fn sent(&mut self, now: Duration) {
        self.next_send = self.interval.and_then(|interval| now.checked_add(interval));
    }
}
