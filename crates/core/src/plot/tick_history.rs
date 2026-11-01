use std::collections::VecDeque;
use std::time::{Duration, Instant};

const SAMPLE_INTERVAL: Duration = Duration::from_millis(500);
const HISTORY: Duration = Duration::from_secs(15 * 60);

struct Sample {
    start: Instant,
    end: Instant,
    ticks: u64,
}

impl Sample {
    fn rate(&self) -> f64 {
        self.ticks as f64 / self.end.duration_since(self.start).as_secs_f64()
    }
}

pub struct TickRateReport {
    pub ten_seconds: f32,
    pub one_minute: f32,
    pub five_minutes: f32,
    pub fifteen_minutes: f32,
}

pub struct TickHistory {
    samples: VecDeque<Sample>,
    current: Sample,
}

impl TickHistory {
    pub fn new(now: Instant) -> TickHistory {
        TickHistory {
            samples: VecDeque::new(),
            current: Sample {
                start: now,
                end: now,
                ticks: 0,
            },
        }
    }

    pub fn record(&mut self, ticks: u64) {
        self.current.ticks += ticks;
    }

    pub fn sample(&mut self, now: Instant) {
        self.current.end = now;
        if now.duration_since(self.current.start) < SAMPLE_INTERVAL {
            return;
        }
        let finished = std::mem::replace(
            &mut self.current,
            Sample {
                start: now,
                end: now,
                ticks: 0,
            },
        );
        self.samples.push_back(finished);
        while self
            .samples
            .front()
            .is_some_and(|sample| now.duration_since(sample.end) > HISTORY)
        {
            self.samples.pop_front();
        }
    }

    pub fn report(&self, now: Instant) -> Option<TickRateReport> {
        if self.samples.is_empty() {
            return None;
        }
        Some(TickRateReport {
            ten_seconds: self.rate(now, Duration::from_secs(10)),
            one_minute: self.rate(now, Duration::from_secs(60)),
            five_minutes: self.rate(now, Duration::from_secs(5 * 60)),
            fifteen_minutes: self.rate(now, HISTORY),
        })
    }

    fn rate(&self, now: Instant, window: Duration) -> f32 {
        let mut ticks = 0.0;
        let mut seconds = 0.0;
        for sample in self.samples.iter().rev() {
            let age = now.duration_since(sample.end);
            if age >= window {
                break;
            }
            let length = sample.end.duration_since(sample.start);
            let covered = length.min(window - age).as_secs_f64();
            ticks += sample.rate() * covered;
            seconds += covered;
        }
        if seconds == 0.0 {
            return 0.0;
        }
        (ticks / seconds) as f32
    }
}
