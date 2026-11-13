use std::collections::VecDeque;
use std::time::{Duration, Instant};

const SAMPLE_LENGTH: Duration = Duration::from_millis(500);
const RECENT_LENGTH: Duration = Duration::from_secs(2);
const HISTORY_LENGTH: Duration = Duration::from_secs(15 * 60);

struct Sample {
    end: Instant,
    length: Duration,
    ticks: u64,
}

#[derive(Default)]
struct Measured {
    ticks: f64,
    covered: Duration,
}

impl Measured {
    fn rate(&self) -> Option<f64> {
        (!self.covered.is_zero()).then(|| self.ticks / self.covered.as_secs_f64())
    }
}

#[derive(Debug)]
pub struct TickRateReport {
    pub ten_seconds: Option<f64>,
    pub one_minute: Option<f64>,
    pub five_minutes: Option<f64>,
    pub fifteen_minutes: f64,
}

pub struct TickHistory {
    samples: VecDeque<Sample>,
    sample_start: Instant,
    sample_ticks: u64,
}

impl TickHistory {
    pub fn new(now: Instant) -> TickHistory {
        TickHistory {
            samples: VecDeque::new(),
            sample_start: now,
            sample_ticks: 0,
        }
    }

    pub fn record(&mut self, ticks: u64) {
        self.sample_ticks += ticks;
    }

    pub fn is_sample_due(&self, now: Instant) -> bool {
        now.duration_since(self.sample_start) >= SAMPLE_LENGTH
    }

    pub fn sample(&mut self, now: Instant) {
        self.add(
            now,
            now.saturating_duration_since(self.sample_start),
            self.sample_ticks,
        );
    }

    fn add(&mut self, end: Instant, length: Duration, ticks: u64) {
        self.sample_start = end;
        self.sample_ticks = 0;
        if length.is_zero() {
            return;
        }
        self.samples.push_back(Sample { end, length, ticks });
        while self
            .samples
            .front()
            .is_some_and(|sample| end.duration_since(sample.end) >= HISTORY_LENGTH)
        {
            self.samples.pop_front();
        }
    }

    pub fn restart(&mut self, now: Instant) {
        self.sample_start = now;
        self.sample_ticks = 0;
    }

    pub fn recent_rate(&self, now: Instant) -> Option<f64> {
        self.measure(now, RECENT_LENGTH).rate()
    }

    pub fn report(&self, now: Instant) -> Option<TickRateReport> {
        let rate = |length| self.measure(now, length).rate();
        Some(TickRateReport {
            ten_seconds: rate(Duration::from_secs(10)),
            one_minute: rate(Duration::from_secs(60)),
            five_minutes: rate(Duration::from_secs(5 * 60)),
            fifteen_minutes: rate(HISTORY_LENGTH)?,
        })
    }

    fn measure(&self, now: Instant, length: Duration) -> Measured {
        let mut measured = Measured::default();
        for sample in self.samples.iter().rev() {
            let age = now.saturating_duration_since(sample.end);
            if age >= length {
                break;
            }
            let covered = sample.length.min(length - age);
            measured.ticks +=
                sample.ticks as f64 * covered.as_secs_f64() / sample.length.as_secs_f64();
            measured.covered += covered;
        }
        measured
    }
}
