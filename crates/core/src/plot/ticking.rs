use super::Plot;
use mchprs_save_data::plot_data::{Tps, WorldSendRate};
use mchprs_world::tick_schedule::Step;
use mchprs_world::{SendRate, TickRate, World};
use std::time::{Duration, Instant};

const TICK_SLICE: Duration = Duration::from_millis(10);
const SLEEP_LIMIT: Duration = Duration::from_millis(10);
const IDLE_SLEEP: Duration = Duration::from_millis(50);

pub(super) fn send_rate(world_send_rate: WorldSendRate) -> SendRate {
    SendRate::per_second(f64::from(world_send_rate.0))
}

impl Plot {
    pub(super) fn update_tick_rate(&mut self) {
        if !self.advance_requests.is_empty() {
            return;
        }
        let rate = if self.players.is_empty() {
            TickRate::Paused
        } else {
            match self.tps {
                Tps::Limited(tps) => TickRate::per_second(f64::from(tps)),
                Tps::Unlimited => TickRate::Unlimited,
            }
        };
        self.set_tick_rate(rate);
    }

    pub(super) fn set_tick_rate(&mut self, rate: TickRate) {
        self.tick_schedule.set_rate(rate, Instant::now());
        if let Some(redpiler) = &mut self.redpiler {
            redpiler.backend.set_tick_rate(rate);
        }
    }

    pub(super) fn update_send_rate(&mut self) {
        let rate = send_rate(self.world.world_send_rate);
        self.send_schedule.set_rate(rate);
        if let Some(redpiler) = &mut self.redpiler {
            redpiler.backend.set_send_rate(rate);
        }
    }

    pub(super) fn tick_interpreted(&mut self) {
        let now = Instant::now();
        if let Step::Run(due) = self.tick_schedule.next(now) {
            let deadline = now + TICK_SLICE;
            let mut completed = 0;
            while completed < due {
                self.tick();
                completed += 1;
                if Instant::now() >= deadline {
                    break;
                }
            }
            self.tick_schedule.complete(completed);
            self.tick_history.record(completed);
        }
        self.report_advance(self.tick_schedule.ticks_owed());
    }

    fn tick(&mut self) {
        self.world
            .to_be_ticked
            .sort_by_key(|e| (e.ticks_left, e.tick_priority));
        for pending in &mut self.world.to_be_ticked {
            pending.ticks_left = pending.ticks_left.saturating_sub(1);
        }
        while self.world.to_be_ticked.first().map_or(1, |e| e.ticks_left) == 0 {
            let entry = self.world.to_be_ticked.remove(0);
            mchprs_redstone::tick(self.world.get_block(entry.pos), &mut self.world, entry.pos);
        }
    }

    pub(super) fn loop_wait(&mut self) -> Duration {
        if self.players.is_empty() {
            return IDLE_SLEEP;
        }
        let now = Instant::now();
        let until_send = self.send_schedule.until_send(now).unwrap_or(Duration::MAX);
        let wait = until_send.min(SLEEP_LIMIT);
        if self.redpiler.is_some() {
            return wait;
        }
        match self.tick_schedule.next(now) {
            Step::Run(_) => Duration::ZERO,
            Step::WaitFor(until_tick) => wait.min(until_tick),
            Step::WaitForRequest => wait,
        }
    }
}
