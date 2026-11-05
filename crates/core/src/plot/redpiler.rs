use super::compile::{CompileJob, Started};
use super::Plot;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_redpiler::backend::CompilePlan;
use mchprs_redpiler::block_map::BlockMap;
use mchprs_redpiler::circuit::Circuit;
use mchprs_redpiler::compile::Stage;
use mchprs_redpiler::session::{Session, SessionContext, SessionError, Update};
use mchprs_redpiler_backends::BackendSetup;
use mchprs_redstone::pace::{Goal, SendRate, TickRate};
use mchprs_redstone::tick_queue::{TickEntry, TickQueue};
use mchprs_save_data::plot_data::{Tps, WorldSendRate};
use std::mem;
use std::time::{Duration, Instant};
use tracing::{debug, error, trace, warn};

const SESSION_TIMEOUT: Duration = Duration::from_secs(10);

pub struct Redpiler {
    pub setup: BackendSetup,
    state: State,
    plot_ticks: Vec<TickEntry>,
    started: Instant,
    tick_rate: TickRate,
    pub(super) behind: bool,
    pub(super) advancing: u64,
}

enum State {
    Compiling(CompileJob),
    Running {
        blocks: BlockMap,
        session: Box<dyn Session>,
    },
}

impl Redpiler {
    pub fn is_compiling(&self) -> bool {
        matches!(self.state, State::Compiling(_))
    }

    pub fn stage(&self) -> Option<Stage> {
        match &self.state {
            State::Compiling(job) => job.stage(),
            State::Running { .. } => None,
        }
    }

    pub fn circuit(&self) -> Option<&Circuit> {
        match &self.state {
            State::Compiling(_) => None,
            State::Running { blocks, .. } => Some(blocks.circuit()),
        }
    }

    pub fn use_block(&mut self, position: BlockPos) -> bool {
        let State::Running { blocks, session } = &mut self.state else {
            return false;
        };
        let Some(drive) = blocks.use_block(position) else {
            return false;
        };
        session.drive(drive);
        true
    }

    pub fn set_pressure_plate(&mut self, position: BlockPos, pressed: bool) {
        if let State::Running { blocks, session } = &mut self.state
            && let Some(drive) = blocks.pressure_plate(position, pressed)
        {
            session.drive(drive);
        }
    }

    pub fn advance(&mut self, goal: Option<Goal>) {
        if let State::Running { session, .. } = &mut self.state {
            session.advance(goal);
        }
    }

    fn set_tick_rate(&mut self, rate: TickRate) {
        if let State::Running { session, .. } = &mut self.state
            && self.tick_rate != rate
        {
            self.tick_rate = rate;
            session.set_tick_rate(rate);
        }
    }
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
        let Some(Redpiler {
            state: State::Running { session, .. },
            ..
        }) = &mut self.redpiler
        else {
            return;
        };
        match session.flush(SESSION_TIMEOUT) {
            Ok(update) => self.apply_update(update),
            Err(SessionError::Unresponsive) => {
                warn!("Redpiler didn't reach a tick boundary in time, so the world lags behind it");
            }
            Err(error) => self.fail_redpiler(error),
        }
    }

    pub(super) fn poll_redpiler(&mut self) {
        let rate = self.session_tick_rate();
        let Some(redpiler) = &mut self.redpiler else {
            return;
        };
        redpiler.set_tick_rate(rate);
        match &mut redpiler.state {
            State::Compiling(job) => {
                let progressed = job.progressed();
                let finished = job.finished();
                if progressed {
                    trace!("Redpiler compile stage {:?}", redpiler.stage());
                    self.refresh_scoreboard();
                }
                match finished {
                    None => {}
                    Some(Ok(started)) => self.run_redpiler(started),
                    Some(Err(error)) => self.fail_compile(&error),
                }
            }
            State::Running { session, .. } => match session.poll() {
                Ok(Some(update)) => self.apply_update(update),
                Ok(None) => {}
                Err(error) => self.fail_redpiler(error),
            },
        }
    }

    fn session_tick_rate(&self) -> TickRate {
        if self.players.is_empty() {
            TickRate::Paused
        } else {
            tick_rate(self.tps)
        }
    }

    fn apply_update(&mut self, update: Update) {
        let Some(Redpiler {
            state: State::Running { blocks, .. },
            behind,
            advancing,
            ..
        }) = &mut self.redpiler
        else {
            return;
        };
        blocks.apply(&update.changes, &mut self.world);
        *behind = update.behind;
        *advancing = update.advancing;
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
            self.advances.ran(advanced);
        }
    }

    fn run_redpiler(&mut self, started: Started) {
        let Started {
            blocks,
            mut session,
        } = started;
        let rate = self.session_tick_rate();
        for (position, pressed) in blocks.pressure_plates() {
            let stood_on = self.are_players_on_block(position);
            if pressed != stood_on
                && let Some(drive) = blocks.pressure_plate(position, stood_on)
            {
                session.drive(drive);
            }
        }
        let Some(redpiler) = &mut self.redpiler else {
            return;
        };
        let message = format!(
            "Redpiler compiled {} nodes for the {} backend in {:?}.",
            blocks.circuit().len(),
            redpiler.setup.backend().name(),
            redpiler.started.elapsed()
        );
        redpiler.state = State::Running { blocks, session };
        redpiler.set_tick_rate(rate);
        debug!("{message}");
        self.broadcast(&message);
        self.reset_timings();
        self.refresh_scoreboard();
    }

    fn fail_compile(&mut self, error: &anyhow::Error) {
        error!("Redpiler failed to compile: {error}");
        if let Some(redpiler) = self.redpiler.take() {
            self.abandon_compile(redpiler.plot_ticks);
        }
        self.broadcast(&format!("Redpiler failed and was stopped: {error}."));
        self.end_redpiler();
    }

    fn fail_redpiler(&mut self, error: SessionError) {
        error!("Redpiler failed: {error}");
        self.redpiler = None;
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
        self.redpiler = Some(Redpiler {
            setup,
            state: State::Compiling(job),
            plot_ticks: mem::take(&mut self.world.ticks).into_entries(),
            started: Instant::now(),
            tick_rate: TickRate::Paused,
            behind: false,
            advancing: 0,
        });
        self.reset_timings();
        self.refresh_scoreboard();
        Ok(())
    }

    pub fn stop_redpiler(&mut self, reset: bool, update: bool) {
        let Some(redpiler) = self.redpiler.take() else {
            return;
        };
        match redpiler.state {
            State::Compiling(job) => {
                debug!("Cancelling the redpiler compile");
                drop(job);
                self.abandon_compile(redpiler.plot_ticks);
                self.end_redpiler();
            }
            State::Running {
                mut blocks,
                session,
            } => {
                debug!("Stopping redpiler");
                match session.stop(reset, SESSION_TIMEOUT) {
                    Ok(stopped) => {
                        if !redpiler.plot_ticks.is_empty() {
                            self.world.mark_unsaved();
                        }
                        blocks.record(&stopped.update.changes);
                        blocks.finish(&stopped.snapshot, &mut self.world);
                        self.record_ticks(&stopped.update);
                        self.end_redpiler();
                        if reset {
                            self.reset_pressure_plates();
                        }
                    }
                    Err(error) => self.fail_redpiler(error),
                }
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
        self.redpiler.as_ref().is_some_and(Redpiler::is_compiling)
    }
}
