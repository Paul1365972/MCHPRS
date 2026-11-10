use crate::ticks::{Ran, Stop};
use crate::{Event, Run};
use std::time::{Duration, Instant};

const SLICE: Duration = Duration::from_millis(10);
const CHUNK_LIMIT: u64 = 50_000;
const CATCH_UP_LIMIT: Duration = Duration::from_secs(2);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Chunk {
    Run(u64),
    Settle(u64),
}

/// Turns a rate into the ticks that are due and runs them in chunks. The stepper uses it, and the
/// plot uses it for the world of a plot with nothing compiled.
#[derive(Debug, Clone, Default)]
pub struct Schedule {
    current: Option<Current>,
    tick_time: Option<Duration>,
    behind: bool,
}

#[derive(Debug, Clone, Copy)]
struct Current {
    run: Run,
    since: Instant,
    ran: u64,
}

impl Current {
    fn has_goal(&self) -> bool {
        matches!(self.run, Run::Ticks(_) | Run::UntilSettled)
    }

    fn next_tick(&self) -> Option<Instant> {
        let Run::Every(interval) = self.run else {
            return Some(self.since);
        };
        let elapsed = interval.as_nanos().saturating_mul(u128::from(self.ran));
        let elapsed = Duration::from_nanos(u64::try_from(elapsed).ok()?);
        self.since.checked_add(elapsed)
    }
}

impl Schedule {
    pub fn pause<A>(&mut self, events: &mut Vec<Event<A>>) {
        self.interrupt(events);
        self.current = None;
        self.behind = false;
    }

    pub fn set<A>(&mut self, run: Run, now: Instant, events: &mut Vec<Event<A>>) {
        self.interrupt(events);
        self.current = Some(Current {
            run,
            since: now,
            ran: 0,
        });
        self.behind = false;
        if run == Run::Ticks(0) {
            self.finish(events);
        }
    }

    pub fn next_tick(&self) -> Option<Instant> {
        self.current?.next_tick()
    }

    pub fn is_behind(&self) -> bool {
        self.behind
    }

    pub fn run<A, E>(
        &mut self,
        now: Instant,
        events: &mut Vec<Event<A>>,
        ticks: impl FnOnce(Chunk) -> Result<Ran, E>,
    ) -> Result<(u64, Instant), E> {
        let Some(chunk) = self.chunk(now) else {
            return Ok((0, now));
        };
        let ran = ticks(chunk)?;
        let end = Instant::now();
        self.ran(ran, end - now, end, events);
        Ok((ran.ticks, end))
    }

    fn chunk(&mut self, now: Instant) -> Option<Chunk> {
        let limit = self.chunk_limit();
        let current = self.current.as_mut()?;
        Some(match current.run {
            Run::Every(interval) => {
                let next_tick = current.next_tick()?;
                if now.saturating_duration_since(next_tick) > CATCH_UP_LIMIT {
                    current.since = now;
                    current.ran = 0;
                    self.behind = true;
                }
                let elapsed = now.saturating_duration_since(current.since).as_nanos();
                let due = elapsed / interval.as_nanos().max(1) + 1;
                let due = u64::try_from(due)
                    .unwrap_or(u64::MAX)
                    .saturating_sub(current.ran);
                if due == 0 {
                    return None;
                }
                Chunk::Run(due.min(limit))
            }
            Run::Unlimited => Chunk::Run(limit),
            Run::Ticks(total) => Chunk::Run(total.saturating_sub(current.ran).min(limit)),
            Run::UntilSettled => Chunk::Settle(limit),
        })
    }

    fn chunk_limit(&self) -> u64 {
        let Some(tick_time) = self.tick_time else {
            return 1;
        };
        let limit = SLICE.as_nanos() / tick_time.as_nanos().max(1);
        u64::try_from(limit).map_or(CHUNK_LIMIT, |limit| limit.clamp(1, CHUNK_LIMIT))
    }

    fn ran<A>(&mut self, ran: Ran, elapsed: Duration, end: Instant, events: &mut Vec<Event<A>>) {
        if ran.ticks > 0 {
            let tick_time = elapsed.as_nanos() / u128::from(ran.ticks);
            self.tick_time = Some(Duration::from_nanos(
                u64::try_from(tick_time).unwrap_or(u64::MAX),
            ));
        }
        let Some(current) = &mut self.current else {
            return;
        };
        current.ran += ran.ticks;
        let finished = match current.run {
            Run::Ticks(total) => current.ran >= total,
            Run::UntilSettled => ran.stop == Some(Stop::Settled),
            Run::Every(_) | Run::Unlimited => false,
        };
        if finished {
            self.finish(events);
        } else if current.next_tick().is_some_and(|next_tick| next_tick > end) {
            self.behind = false;
        }
        if let Some(Stop::Breakpoint(hit)) = ran.stop {
            self.pause(events);
            events.push(Event::Halted(hit));
        }
    }

    fn interrupt<A>(&mut self, events: &mut Vec<Event<A>>) {
        if self.current.is_some_and(|current| current.has_goal()) {
            events.push(Event::RunInterrupted);
        }
    }

    fn finish<A>(&mut self, events: &mut Vec<Event<A>>) {
        events.push(Event::RunFinished);
        self.current = None;
        self.behind = false;
    }
}
