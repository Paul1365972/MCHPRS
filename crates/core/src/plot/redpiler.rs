use super::Plot;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::region::Region;
use mchprs_commands::flag::FlagValues;
use mchprs_redpiler::backend::{Backend, CompilePlan};
use mchprs_redpiler::circuit::Circuit;
use mchprs_redpiler::compile::Stage;
use mchprs_redpiler::simulation::Simulation;
use mchprs_redstone::tick_queue::{TickEntry, TickQueue};
use mchprs_redstone::RedstoneWorld;
use mchprs_save_data::plot_data::{Tps, WorldSendRate};
use mchprs_worker::{Event, SendRate, TickRate, Worker};
use std::mem;
use std::sync::Arc;
use std::time::Instant;
use tracing::{debug, error, trace, warn};

pub struct Redpiler {
    pub backend: &'static dyn Backend,
    pub flags: FlagValues,
    pub worker: Worker,
    pub stage: Option<Stage>,
    pub circuit: Option<Arc<Circuit>>,
    pub plot_ticks: Vec<TickEntry>,
    pub started: Instant,
    pub report_requested: bool,
    pub advanced: u64,
    pub paused: bool,
}

pub fn tick_rate(tps: Tps) -> TickRate {
    match tps {
        Tps::Limited(tps) => TickRate::PerSecond(f64::from(tps)),
        Tps::Unlimited => TickRate::Unlimited,
    }
}

pub fn send_rate(rate: WorldSendRate) -> SendRate {
    SendRate {
        per_second: f64::from(rate.0),
    }
}

impl Plot {
    pub fn sync_redpiler(&mut self) -> Option<Vec<TickEntry>> {
        let redpiler = self.redpiler.as_mut()?;
        if redpiler.circuit.is_none() {
            return Some(redpiler.plot_ticks.clone());
        }
        let mut pending_ticks = None;
        for event in redpiler.worker.sync() {
            if let Some(ticks) = self.apply_redpiler_event(event) {
                pending_ticks = Some(ticks);
            }
        }
        pending_ticks
    }

    pub(super) fn handle_redpiler_events(&mut self) {
        let Some(redpiler) = &mut self.redpiler else {
            return;
        };
        let paused = self.players.is_empty();
        if redpiler.paused != paused {
            redpiler.paused = paused;
            let rate = match paused {
                true => TickRate::Paused,
                false => tick_rate(self.tps),
            };
            redpiler.worker.set_tick_rate(rate);
        }
        if !redpiler.report_requested {
            redpiler.worker.report_ticks();
            redpiler.report_requested = true;
        }
        for event in redpiler.worker.poll() {
            self.apply_redpiler_event(event);
        }
    }

    fn apply_redpiler_event(&mut self, event: Event) -> Option<Vec<TickEntry>> {
        match event {
            Event::Progress(stage) => {
                trace!("Redpiler compile stage {stage:?}");
                self.redpiler.as_mut()?.stage = Some(stage);
                self.refresh_scoreboard();
            }
            Event::Started(circuit) => {
                let redpiler = self.redpiler.as_mut()?;
                let message = format!(
                    "Redpiler compiled {} nodes for the {} engine in {:?}.",
                    circuit.len(),
                    redpiler.backend.name(),
                    redpiler.started.elapsed()
                );
                redpiler.stage = None;
                redpiler.circuit = Some(circuit);
                debug!("{message}");
                self.broadcast(&message);
                self.reset_timings();
                self.refresh_scoreboard();
            }
            Event::Ticked(report) => {
                let redpiler = self.redpiler.as_mut()?;
                redpiler.report_requested = false;
                redpiler.advanced = report.advanced;
                if report.ticks > 0 {
                    self.timings.tickn(report.ticks);
                    self.tick_time = Some(report.ticking.div_f64(report.ticks as f64));
                }
            }
            Event::Batch(changes) => {
                changes.apply(&mut self.world);
                self.publish_world();
            }
            Event::Advanced { ran, settled } => {
                self.redpiler.as_mut()?.advanced = 0;
                self.advances.worker_ran(ran, settled);
            }
            Event::Rejected { position, error } => {
                warn!("Redpiler rejected the input at {position}: {error}");
            }
            Event::Synced { changes, ticks } => {
                changes.apply(&mut self.world);
                return Some(ticks);
            }
            Event::Stopped { changes, ticks } => {
                changes.apply(&mut self.world);
                for tick in ticks {
                    self.world
                        .schedule_tick(tick.position, tick.ticks_left, tick.priority);
                }
                self.end_redpiler();
            }
            Event::Cancelled => {
                self.abandon_compile();
                self.end_redpiler();
            }
            Event::Failed(error) => {
                error!("Redpiler failed: {error}");
                self.broadcast(&format!("Redpiler failed and was stopped: {error}"));
                if self
                    .redpiler
                    .as_ref()
                    .is_some_and(|redpiler| redpiler.circuit.is_none())
                {
                    self.abandon_compile();
                }
                self.end_redpiler();
            }
        }
        None
    }

    fn abandon_compile(&mut self) {
        let Some(redpiler) = &mut self.redpiler else {
            return;
        };
        self.world.ticks = TickQueue::new(mem::take(&mut redpiler.plot_ticks));
        self.reset_pressure_plates();
    }

    pub(super) fn reset_pressure_plates(&mut self) {
        let region = self.world.region();
        for pos in self
            .world
            .occupied_regions(region)
            .into_iter()
            .flat_map(Region::positions)
        {
            if let Some(&mut powered) = self.world.block(pos).get_pressure_plate_powered()
                && powered != self.are_players_on_block(pos)
            {
                mchprs_redstone::set_pressure_plate(&mut self.world, pos, !powered);
            }
        }
    }

    fn end_redpiler(&mut self) {
        self.redpiler = None;
        self.advances.worker_ended();
        self.publish_world();
        self.reset_timings();
        self.refresh_scoreboard();
    }

    pub fn start_redpiler(
        &mut self,
        backend: &'static dyn Backend,
        flags: FlagValues,
        plan: CompilePlan,
    ) {
        debug!("Starting redpiler with the {} engine", backend.name());
        self.reset_pressure_plates();
        let region = self.world.region();
        let blocks = self.world.copy_blocks();
        let ticks = mem::take(&mut self.world.ticks).into_entries();
        let compile_ticks = ticks.clone();
        let worker = Worker::spawn(
            tick_rate(self.tps),
            send_rate(self.world.world_send_rate),
            move |progress| {
                let circuit = mchprs_redpiler::compile::compile(
                    &blocks,
                    region,
                    &compile_ticks,
                    &plan.extract,
                    &plan.passes,
                    progress,
                )?;
                Ok(Simulation::new((plan.build)(Arc::new(circuit))?))
            },
        );
        self.redpiler = Some(Redpiler {
            backend,
            flags,
            worker,
            stage: None,
            circuit: None,
            plot_ticks: ticks,
            started: Instant::now(),
            report_requested: false,
            advanced: 0,
            paused: false,
        });
        self.refresh_scoreboard();
    }

    pub fn stop_redpiler(&mut self, restore: bool, update: bool) {
        let Some(redpiler) = &mut self.redpiler else {
            return;
        };
        if redpiler.circuit.is_none() {
            debug!("Cancelling the redpiler compile");
            self.abandon_compile();
            self.end_redpiler();
        } else {
            debug!("Stopping redpiler");
            for event in redpiler.worker.stop(restore) {
                self.apply_redpiler_event(event);
            }
            if restore {
                self.reset_pressure_plates();
            }
        }
        if update {
            let region = self.world.region();
            for position in self
                .world
                .occupied_regions(region)
                .into_iter()
                .flat_map(Region::positions)
            {
                let block = self.world.block(position);
                mchprs_redstone::update(block, &mut self.world, position);
            }
        }
        self.publish_world();
    }

    pub fn reset_redpiler(&mut self) {
        self.stop_redpiler(false, false);
    }

    pub fn is_compiling_redpiler(&self) -> bool {
        self.redpiler
            .as_ref()
            .is_some_and(|redpiler| redpiler.circuit.is_none())
    }
}
