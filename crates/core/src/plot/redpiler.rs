use super::scoreboard::RedpilerState;
use super::Plot;
use crate::player::PacketSender;
use mchprs_redpiler::backend::direct;
use mchprs_redpiler::{Backend, Batch, CompileProgress, CompilerOptions, Netlist};
use mchprs_world::{for_each_block_mut_optimized, TickEntry, TickRate, World};
use std::thread;
use std::time::{Duration, Instant};
use tracing::{debug, error};

const KEEP_ALIVE_POLL: Duration = Duration::from_millis(20);

pub(super) struct Redpiler {
    pub(super) backend: Box<dyn Backend>,
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
        let progress = CompileProgress::default();
        let started = thread::scope(|s| {
            let handle = s.spawn(|| {
                let ticks = &self.world.to_be_ticked;
                let netlist =
                    mchprs_redpiler::compile(&self.world, bounds, ticks, &options, &progress);
                start_backend(&netlist, ticks, &options, &progress)
            });
            let mut shown = None;
            while !handle.is_finished() {
                let status = progress.status();
                if status != shown {
                    shown = status;
                    self.scoreboard
                        .set_redpiler_state(&self.players, RedpilerState::Compiling(status));
                }
                for player in &mut self.players {
                    player.update();
                }
                thread::sleep(KEEP_ALIVE_POLL);
            }
            handle.join()
        });

        let backend = match started {
            Ok(Ok(backend)) => backend,
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
        self.redpiler = Some(Redpiler { backend, options });
        self.update_send_rate();
        if self.advance_requests.is_empty() {
            self.update_tick_rate();
        } else {
            self.set_tick_rate(TickRate::Ticks(self.advance_ticks_owed));
        }
        self.scoreboard
            .set_redpiler_state(&self.players, RedpilerState::Running);

        self.restart_ticking();
    }

    pub(super) fn stop_redpiler(&mut self) {
        let Some(redpiler) = self.redpiler.take() else {
            return;
        };
        debug!("Stopping redpiler");
        match redpiler.backend.stop() {
            Ok(snapshot) => {
                self.apply_batch(snapshot.changes);
                self.world.to_be_ticked.extend(snapshot.pending_ticks);
                self.fail_advance_requests("Redpiler stopped before the advance completed.");
            }
            Err(_) => self.report_crash(),
        }

        if redpiler.options.update {
            let (first_pos, second_pos) = self.world.get_corners();
            for_each_block_mut_optimized(&mut self.world, first_pos, second_pos, |world, pos| {
                let block = world.get_block(pos);
                mchprs_redstone::update(block, world, pos);
            });
        }

        self.show_redpiler_stopped();
        self.restart_ticking();
    }

    pub(super) fn receive_batch(&mut self, timeout: Duration) {
        let Some(redpiler) = &mut self.redpiler else {
            return;
        };
        match redpiler.backend.next_batch(timeout) {
            Ok(batch) => self.apply_batch(batch),
            Err(_) => self.discard_crashed_redpiler(),
        }
    }

    pub(super) fn pull_snapshot(&mut self) -> Option<Vec<TickEntry>> {
        let redpiler = self.redpiler.as_mut()?;
        match redpiler.backend.snapshot() {
            Ok(snapshot) => {
                self.apply_batch(snapshot.changes);
                Some(snapshot.pending_ticks)
            }
            Err(_) => {
                self.discard_crashed_redpiler();
                None
            }
        }
    }

    pub(super) fn publish_world(&mut self) {
        self.pull_snapshot();
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
        for request in self.advance_requests.drain(..) {
            let message = format!(
                "Plot has been advanced by {} ticks ({:?})",
                request.ticks,
                request.started.elapsed()
            );
            if let Some(player) = self
                .players
                .iter()
                .find(|player| player.uuid == request.player)
            {
                player.send_system_message(&message);
            }
        }
        self.update_tick_rate();
    }

    fn apply_batch(&mut self, batch: Batch) {
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
        for request in self.advance_requests.drain(..) {
            if let Some(player) = self
                .players
                .iter()
                .find(|player| player.uuid == request.player)
            {
                player.send_error_message(message);
            }
        }
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
) -> Result<Box<dyn Backend>, String> {
    let name = options.backend.as_deref().unwrap_or("direct");
    if name == "direct" {
        return Ok(Box::new(direct::start(netlist, ticks, options.io_only)));
    }
    #[cfg(feature = "fpga")]
    {
        let target = mchprs_redpiler_fpga::Target::from_name(name).ok_or_else(|| {
            format!("unknown backend {name}; use direct, phase, verilator or fpga")
        })?;
        mchprs_redpiler_fpga::start(target, netlist, ticks, options.io_only, progress)
            .map(|worker| Box::new(worker) as Box<dyn Backend>)
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
