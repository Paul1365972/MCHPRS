use super::Plot;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::region::Region;
use mchprs_redpiler::backend::CompilePlan;
use mchprs_redpiler::circuit::Circuit;
use mchprs_redpiler::compile::Stage;
use mchprs_redpiler::session::{Event, TickReport, Worker};
use mchprs_redpiler::simulation::Simulation;
use mchprs_redpiler_backends::BackendSetup;
use mchprs_redstone::pace::{SendRate, TickRate};
use mchprs_redstone::tick_queue::{TickEntry, TickQueue};
use mchprs_redstone::RedstoneWorld;
use mchprs_save_data::plot_data::{Tps, WorldSendRate};
use std::mem;
use std::sync::Arc;
use std::time::Instant;
use tracing::{debug, error, trace, warn};

pub struct Redpiler {
    pub setup: BackendSetup,
    pub worker: Worker,
    pub stage: Option<Stage>,
    pub circuit: Option<Arc<Circuit>>,
    pub plot_ticks: Vec<TickEntry>,
    pub started: Instant,
    pub paused: bool,
    pub report_requested: bool,
    pub advanced: u64,
    pub behind: bool,
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
    pub fn sync_redpiler(&mut self) {
        let Some(redpiler) = &mut self.redpiler else {
            return;
        };
        if redpiler.circuit.is_none() {
            return;
        }
        for event in redpiler.worker.sync() {
            self.apply_redpiler_event(event);
        }
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
        for event in redpiler.worker.poll() {
            self.apply_redpiler_event(event);
        }
    }

    fn apply_redpiler_event(&mut self, event: Event) {
        let Some(redpiler) = &mut self.redpiler else {
            return;
        };
        match event {
            Event::Progress(stage) => {
                trace!("Redpiler compile stage {stage:?}");
                redpiler.stage = Some(stage);
                self.refresh_scoreboard();
            }
            Event::Started(circuit) => {
                let message = format!(
                    "Redpiler compiled {} nodes for the {} backend in {:?}.",
                    circuit.len(),
                    redpiler.setup.backend().name(),
                    redpiler.started.elapsed()
                );
                redpiler.stage = None;
                redpiler.circuit = Some(circuit);
                debug!("{message}");
                self.broadcast(&message);
                self.reset_timings();
                self.refresh_scoreboard();
            }
            Event::Ticked(report) => self.apply_tick_report(report),
            Event::Batch(changes) => {
                changes.apply(&mut self.world);
                self.publish_world();
            }
            Event::Advanced(advanced) => {
                redpiler.advanced = 0;
                self.advances.ran(advanced);
            }
            Event::Rejected { position, error } => {
                warn!("Redpiler rejected the input at {position}: {error}");
            }
            Event::Synced(changes) => changes.apply(&mut self.world),
            Event::Stopped { changes, ticks } => {
                if !redpiler.plot_ticks.is_empty() {
                    self.world.mark_unsaved();
                }
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
                let compiling = redpiler.circuit.is_none();
                error!("Redpiler failed: {error}");
                let mut message = format!("Redpiler failed and was stopped: {error}.");
                if compiling {
                    self.abandon_compile();
                } else {
                    match self.restore_saved_world() {
                        Ok(()) => message
                            .push_str(" The plot was restored to its state before compiling."),
                        Err(error) => {
                            error!("Failed to restore the plot after redpiler failed: {error:?}");
                            message.push_str(" The plot couldn't be restored.");
                        }
                    }
                }
                self.broadcast(&message);
                self.end_redpiler();
            }
        }
    }

    fn apply_tick_report(&mut self, report: TickReport) {
        let Some(redpiler) = &mut self.redpiler else {
            return;
        };
        redpiler.report_requested = false;
        redpiler.advanced = report.advanced;
        redpiler.behind = report.behind;
        if report.ticks > 0 {
            self.tick_time = Some(report.ticking.div_f64(report.ticks as f64));
        }
        if !self.players.is_empty() {
            self.tick_history
                .add(Instant::now(), report.period, report.ticks);
        }
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
        self.world.postpone_autosave();
        self.advances.release();
        self.publish_world();
        self.reset_timings();
        self.refresh_scoreboard();
    }

    pub fn start_redpiler(
        &mut self,
        setup: BackendSetup,
        plan: CompilePlan,
    ) -> Result<(), &'static str> {
        self.reset_pressure_plates();
        self.save();
        if self.world.unsaved_since.is_some() {
            return Err("Redpiler didn't start because the plot couldn't be saved.");
        }
        self.end_world_advance();
        debug!("Starting redpiler with {setup}");
        let region = self.world.region();
        let blocks = self.world.copy_blocks();
        let ticks = mem::take(&mut self.world.ticks).into_entries();
        let compile_ticks = ticks.clone();
        let backend = setup.backend();
        let flags = setup.flags().clone();
        let worker = Worker::spawn(
            tick_rate(self.tps),
            send_rate(self.world.world_send_rate),
            move |context| {
                let circuit = mchprs_redpiler::compile::compile(
                    &blocks,
                    region,
                    &compile_ticks,
                    &plan.extract,
                    &plan.passes,
                    context,
                )?;
                let engine = backend.build(&circuit, &flags, context)?;
                Ok(Simulation::new(Arc::new(circuit), engine))
            },
        );
        self.redpiler = Some(Redpiler {
            setup,
            worker,
            stage: None,
            circuit: None,
            plot_ticks: ticks,
            started: Instant::now(),
            paused: false,
            report_requested: false,
            advanced: 0,
            behind: false,
        });
        self.reset_timings();
        self.refresh_scoreboard();
        Ok(())
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
