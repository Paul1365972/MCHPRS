use super::scoreboard::RedpilerState;
use super::Plot;
use crate::player::{PacketSender, Player};
use mchprs_redpiler::backend::direct;
use mchprs_redpiler::{
    BackendFailure, Batch, CompileProgress, CompilerInput, CompilerOptions, Netlist, Worker,
};
use mchprs_world::{TickEntry, TickRate, World};
use std::convert::Infallible;
use std::mem;
use std::sync::mpsc::{self, RecvTimeoutError};
use std::thread;
use std::time::{Duration, Instant};
use tracing::{debug, error};

const KEEP_ALIVE_INTERVAL: Duration = Duration::from_millis(20);

pub(super) struct Redpiler {
    pub(super) worker: Worker,
    pub(super) options: CompilerOptions,
}

pub(super) struct AdvanceRequest {
    player: u128,
    ticks: u64,
    started: Instant,
}

impl Plot {
    pub(super) fn start_redpiler(&mut self, options: CompilerOptions) {
        debug!("Starting redpiler");
        self.scoreboard
            .set_redpiler_state(&self.players, RedpilerState::Compiling(None));
        self.scoreboard
            .set_redpiler_options(&self.players, &options);

        let bounds = self.world.get_corners();
        mchprs_redstone::update_region(&mut self.world, bounds.0, bounds.1);
        self.world.flush_block_changes();
        let progress = CompileProgress::default();
        let started = thread::scope(|scope| {
            let (finished_sender, finished) = mpsc::channel::<Infallible>();
            let compiling = scope.spawn(|| {
                let input = CompilerInput {
                    world: &self.world,
                    bounds,
                    ticks: &self.world.to_be_ticked,
                };
                let netlist = mchprs_redpiler::compile(&input, &options, &progress);
                let started = start_backend(&netlist, input.ticks, &options, &progress);
                drop(finished_sender);
                started
            });
            let mut shown = None;
            while let Err(RecvTimeoutError::Timeout) = finished.recv_timeout(KEEP_ALIVE_INTERVAL) {
                let status = progress.status();
                if status != shown {
                    shown = status;
                    self.scoreboard
                        .set_redpiler_state(&self.players, RedpilerState::Compiling(status));
                }
                for player in &mut self.players {
                    player.update();
                }
            }
            compiling.join()
        });

        let worker = match started {
            Ok(Ok(worker)) => worker,
            Ok(Err(message)) => {
                error!("Redpiler compile failed: {message}");
                self.broadcast_plot_chat_message(&format!("Redpiler compile failed: {message}"));
                self.show_redpiler_stopped();
                return;
            }
            Err(_) => {
                self.broadcast_plot_chat_message("Redpiler compile failed, see the server log.");
                self.show_redpiler_stopped();
                return;
            }
        };

        self.world.to_be_ticked.clear();
        self.scoreboard
            .set_redpiler_state(&self.players, RedpilerState::Running);
        self.redpiler = Some(Redpiler { worker, options });
        self.update_send_rate();
        if self.advance_requests.is_empty() {
            self.update_tick_rate();
        } else {
            self.set_tick_rate(TickRate::Ticks(self.advance_ticks_owed));
        }

        self.restart_ticking();
    }

    pub(super) fn stop_redpiler(&mut self) {
        let Some(redpiler) = self.redpiler.take() else {
            return;
        };
        debug!("Stopping redpiler");
        match redpiler.worker.stop() {
            Ok(snapshot) => {
                self.apply_batch(snapshot.changes);
                self.world.to_be_ticked.extend(snapshot.pending_ticks);
                self.fail_advance_requests("Redpiler stopped before the advance completed.");
            }
            Err(_) => self.report_crash(),
        }

        if redpiler.options.update {
            let (first_pos, second_pos) = self.world.get_corners();
            mchprs_redstone::update_region(&mut self.world, first_pos, second_pos);
        }

        self.show_redpiler_stopped();
        self.restart_ticking();
    }

    pub(super) fn with_worker<T>(
        &mut self,
        operation: impl FnOnce(&mut Worker) -> Result<T, BackendFailure>,
    ) -> Option<T> {
        let redpiler = self.redpiler.as_mut()?;
        match operation(&mut redpiler.worker) {
            Ok(value) => Some(value),
            Err(BackendFailure) => {
                self.discard_crashed_redpiler();
                None
            }
        }
    }

    pub(super) fn receive_batch(&mut self, timeout: Duration) {
        if let Some(Some(batch)) = self.with_worker(|worker| worker.next_batch(timeout)) {
            self.apply_batch(batch);
        }
    }

    pub(super) fn sync_world(&mut self) {
        if let Some(snapshot) = self.with_worker(Worker::snapshot) {
            self.apply_batch(snapshot.changes);
        }
    }

    pub(super) fn publish_world(&mut self) {
        self.sync_world();
        self.world.flush_block_changes();
    }

    pub(super) fn advance(&mut self, player: usize, ticks: u64) {
        let rate = TickRate::Ticks(self.advance_ticks_owed.saturating_add(ticks));
        self.set_tick_rate(rate);
        self.advance_ticks_owed = rate.ticks_owed();
        self.advance_requests.push(AdvanceRequest {
            player: self.players[player].uuid,
            ticks,
            started: Instant::now(),
        });
    }

    pub(super) fn report_advance(&mut self, ticks_owed: u64) {
        self.advance_ticks_owed = ticks_owed;
        if ticks_owed != 0 || self.advance_requests.is_empty() {
            return;
        }
        for request in mem::take(&mut self.advance_requests) {
            if let Some(player) = self.player_by_uuid(request.player) {
                player.send_system_message(&format!(
                    "Plot has been advanced by {} ticks ({:?})",
                    request.ticks,
                    request.started.elapsed()
                ));
            }
        }
        self.update_tick_rate();
    }

    pub(super) fn apply_batch(&mut self, batch: Batch) {
        let ticks_completed = batch.ticks_completed;
        let ticks_owed = batch.ticks_owed;
        let changes_world = batch.changes_world();
        batch.apply(&mut self.world);
        if changes_world {
            self.world.flush_block_changes();
        }
        self.tick_history.record(ticks_completed);
        self.report_advance(ticks_owed);
    }

    fn discard_crashed_redpiler(&mut self) {
        self.redpiler = None;
        self.report_crash();
        self.show_redpiler_stopped();
        self.restart_ticking();
    }

    fn report_crash(&mut self) {
        self.fail_advance_requests("Redpiler crashed before the advance completed.");
        self.broadcast_plot_chat_message("Redpiler crashed, the plot continues without it.");
    }

    fn fail_advance_requests(&mut self, message: &str) {
        self.advance_ticks_owed = 0;
        for request in mem::take(&mut self.advance_requests) {
            if let Some(player) = self.player_by_uuid(request.player) {
                player.send_error_message(message);
            }
        }
    }

    fn player_by_uuid(&self, uuid: u128) -> Option<&Player> {
        self.players.iter().find(|player| player.uuid == uuid)
    }

    fn show_redpiler_stopped(&mut self) {
        self.scoreboard
            .set_redpiler_state(&self.players, RedpilerState::Stopped);
        self.scoreboard
            .set_redpiler_options(&self.players, &Default::default());
    }
}

fn start_backend(
    netlist: &Netlist,
    ticks: &[TickEntry],
    options: &CompilerOptions,
    progress: &CompileProgress,
) -> Result<Worker, String> {
    let name = options.backend.as_deref().unwrap_or("direct");
    if name == "direct" {
        return Ok(direct::start(netlist, ticks, options.reporting()));
    }
    #[cfg(feature = "fpga")]
    {
        let target = name
            .parse::<mchprs_redpiler_fpga::Target>()
            .map_err(|_| format!("unknown backend {name}; use direct, phase, verilator or fpga"))?;
        mchprs_redpiler_fpga::start(target, netlist, ticks, options.io_only, progress)
            .map_err(|error| error.to_string())
    }
    #[cfg(not(feature = "fpga"))]
    {
        let _ = progress;
        Err(format!(
            "unknown backend {name}; phase, verilator and fpga need a server built with --features fpga"
        ))
    }
}
