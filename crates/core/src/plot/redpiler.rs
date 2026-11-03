use super::compile::{CompileJob, Started};
use super::Plot;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_redpiler::backend::CompilePlan;
use mchprs_redpiler::block_map::BlockMap;
use mchprs_redpiler::compile::Stage;
use mchprs_redpiler::session::{Session, SessionContext, SessionError, Update};
use mchprs_redpiler_backends::BackendSetup;
use mchprs_redstone::pace::{Goal, TickRate};
use mchprs_redstone::tick_queue::{TickEntry, TickQueue};
use mchprs_save_data::plot_data::Tps;
use std::mem;
use std::time::{Duration, Instant};
use tracing::{debug, error, trace, warn};

const SESSION_TIMEOUT: Duration = Duration::from_secs(10);

#[derive(Default)]
pub enum Redpiler {
    #[default]
    Stopped,
    Compiling(Compiling),
    Running(Running),
}

pub struct Compiling {
    pub(super) setup: BackendSetup,
    job: CompileJob,
    plot_ticks: Vec<TickEntry>,
    started: Instant,
}

pub struct Running {
    pub(super) setup: BackendSetup,
    blocks: BlockMap,
    session: Box<dyn Session>,
    had_ticks: bool,
    tick_rate: TickRate,
    pub(super) behind: bool,
    pub(super) advanced_ticks: u64,
}

impl Redpiler {
    pub fn is_active(&self) -> bool {
        !matches!(self, Redpiler::Stopped)
    }

    pub fn use_block(&mut self, pos: BlockPos) -> bool {
        let Redpiler::Running(running) = self else {
            return false;
        };
        let Some(drive) = running.blocks.use_block(pos) else {
            return false;
        };
        running.session.drive(drive);
        true
    }

    pub fn set_pressure_plate(&mut self, pos: BlockPos, pressed: bool) {
        if let Redpiler::Running(running) = self
            && let Some(drive) = running.blocks.pressure_plate(pos, pressed)
        {
            running.session.drive(drive);
        }
    }
}

impl Compiling {
    pub fn stage(&self) -> Option<Stage> {
        self.job.stage()
    }
}

impl Running {
    pub fn advance(&mut self, goal: Option<Goal>) {
        self.session.advance(goal);
    }

    fn set_tick_rate(&mut self, rate: TickRate) {
        if self.tick_rate != rate {
            self.tick_rate = rate;
            self.session.set_tick_rate(rate);
        }
    }
}

pub fn tick_rate(tps: Tps) -> TickRate {
    match tps {
        Tps::Limited(tps) => TickRate::per_second(f64::from(tps)),
        Tps::Unlimited => TickRate::Unlimited,
    }
}

impl Plot {
    pub fn sync_redpiler(&mut self) {
        let Redpiler::Running(running) = &mut self.redpiler else {
            return;
        };
        match running.session.flush(SESSION_TIMEOUT) {
            Ok(update) => self.apply_update(update),
            Err(SessionError::Unresponsive) => {
                warn!("Redpiler didn't reach a tick boundary in time, so the world lags behind it");
            }
            Err(error) => self.fail_redpiler(error),
        }
    }

    pub(super) fn poll_redpiler(&mut self) {
        let rate = self.session_tick_rate();
        match &mut self.redpiler {
            Redpiler::Stopped => {}
            Redpiler::Compiling(compiling) => {
                let progressed = compiling.job.progressed();
                let finished = compiling.job.finished();
                if progressed {
                    trace!("Redpiler compile stage {:?}", compiling.stage());
                    self.refresh_scoreboard();
                }
                match finished {
                    None => {}
                    Some(Ok(started)) => self.run_redpiler(started),
                    Some(Err(error)) => self.fail_compile(&error),
                }
            }
            Redpiler::Running(running) => {
                running.set_tick_rate(rate);
                match running.session.poll() {
                    Ok(Some(update)) => self.apply_update(update),
                    Ok(None) => {}
                    Err(error) => self.fail_redpiler(error),
                }
            }
        }
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
        debug!("Starting redpiler with {setup}");
        let session = SessionContext {
            name: format!("p{},{}", self.world.x, self.world.z),
        };
        let job = CompileJob::spawn(
            self.world.copy_blocks(),
            self.world.region(),
            self.world.ticks.entries().to_vec(),
            setup.clone(),
            plan,
            session,
        )
        .map_err(|error| {
            error!("Redpiler couldn't start its compile thread: {error}");
            "Redpiler didn't start because its compile thread couldn't start."
        })?;
        self.end_world_advance();
        self.redpiler = Redpiler::Compiling(Compiling {
            setup,
            job,
            plot_ticks: mem::take(&mut self.world.ticks).into_entries(),
            started: Instant::now(),
        });
        self.reset_timings();
        self.refresh_scoreboard();
        Ok(())
    }

    pub fn stop_redpiler(&mut self, restore: bool, update: bool) {
        match mem::take(&mut self.redpiler) {
            Redpiler::Stopped => return,
            Redpiler::Compiling(compiling) => {
                debug!("Cancelling the redpiler compile");
                drop(compiling.job);
                self.abandon_compile(compiling.plot_ticks);
                self.end_redpiler();
            }
            Redpiler::Running(mut running) => {
                debug!("Stopping redpiler");
                match running.session.stop(restore, SESSION_TIMEOUT) {
                    Ok(stopped) => {
                        if running.had_ticks {
                            self.world.mark_unsaved();
                        }
                        running.blocks.record(&stopped.update.changes);
                        running.blocks.finish(&stopped.snapshot, &mut self.world);
                        self.record_ticks(&stopped.update);
                        self.end_redpiler();
                        if restore {
                            self.reset_pressure_plates();
                        }
                    }
                    Err(error) => self.fail_redpiler(error),
                }
            }
        }
        if update {
            let region = self.world.region();
            for pos in self
                .world
                .occupied_regions(region)
                .into_iter()
                .flat_map(Region::positions)
            {
                let block = self.world.block(pos);
                mchprs_redstone::update(block, &mut self.world, pos);
            }
        }
        self.publish_world();
    }

    pub fn reset_redpiler(&mut self) {
        self.stop_redpiler(false, false);
    }

    pub fn is_compiling_redpiler(&self) -> bool {
        matches!(self.redpiler, Redpiler::Compiling(_))
    }

    fn session_tick_rate(&self) -> TickRate {
        if self.players.is_empty() {
            TickRate::Paused
        } else {
            tick_rate(self.tps)
        }
    }

    fn apply_update(&mut self, update: Update) {
        let Redpiler::Running(running) = &mut self.redpiler else {
            return;
        };
        running.blocks.apply(&update.changes, &mut self.world);
        running.behind = update.behind;
        running.advanced_ticks = update.advanced_ticks;
        self.record_ticks(&update);
        if update.driven || !update.advanced.is_empty() {
            self.publish_world();
        }
    }

    fn record_ticks(&mut self, update: &Update) {
        self.tick_history.record(update.ticks);
        if update.ticks > 0 {
            self.tick_time = Some(update.ticking.div_f64(update.ticks as f64));
        }
        for &advanced in &update.advanced {
            self.record_advance(advanced);
        }
    }

    fn run_redpiler(&mut self, started: Started) {
        let Started {
            blocks,
            mut session,
        } = started;
        let rate = self.session_tick_rate();
        for (pos, pressed) in blocks.pressure_plates() {
            let stood_on = self.are_players_on_block(pos);
            if pressed != stood_on
                && let Some(drive) = blocks.pressure_plate(pos, stood_on)
            {
                session.drive(drive);
            }
        }
        let Redpiler::Compiling(compiling) = mem::take(&mut self.redpiler) else {
            return;
        };
        let message = format!(
            "Redpiler compiled {} nodes for the {} backend in {:?}.",
            blocks.circuit().len(),
            compiling.setup.backend().name(),
            compiling.started.elapsed()
        );
        let mut running = Running {
            setup: compiling.setup,
            blocks,
            session,
            had_ticks: !compiling.plot_ticks.is_empty(),
            tick_rate: TickRate::Paused,
            behind: false,
            advanced_ticks: 0,
        };
        running.set_tick_rate(rate);
        self.redpiler = Redpiler::Running(running);
        debug!("{message}");
        self.broadcast(&message);
        self.reset_timings();
        self.refresh_scoreboard();
    }

    fn fail_compile(&mut self, error: &anyhow::Error) {
        error!("Redpiler failed to compile: {error}");
        if let Redpiler::Compiling(compiling) = mem::take(&mut self.redpiler) {
            self.abandon_compile(compiling.plot_ticks);
        }
        self.broadcast(&format!("Redpiler failed and was stopped: {error}."));
        self.end_redpiler();
    }

    fn fail_redpiler(&mut self, error: SessionError) {
        error!("Redpiler failed: {error}");
        self.redpiler = Redpiler::Stopped;
        let mut message = format!("Redpiler failed and was stopped: {error}.");
        match self.restore_saved_world() {
            Ok(()) => message.push_str(" The plot was restored to its state before compiling."),
            Err(error) => {
                error!("Failed to restore the plot after redpiler failed: {error:?}");
                message.push_str(" The plot couldn't be restored.");
            }
        }
        self.broadcast(&message);
        self.end_redpiler();
    }

    fn abandon_compile(&mut self, plot_ticks: Vec<TickEntry>) {
        self.world.ticks = TickQueue::new(plot_ticks);
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
        self.redpiler = Redpiler::Stopped;
        self.release_advance();
        self.publish_world();
        self.reset_timings();
        self.refresh_scoreboard();
    }
}
