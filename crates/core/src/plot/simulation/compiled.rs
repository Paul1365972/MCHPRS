use super::{Before, Compiled, Compiling, PlotSimulation};
use crate::plot::compile::{Compilation, CompileJob};
use crate::plot::Plot;
use mchprs_backends::Setup;
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::BlockPos;
use mchprs_redstone::input::Input;
use mchprs_redstone::tick_queue::TickQueue;
use mchprs_redstone::{plate, Aabb};
use mchprs_simulation::backend::CompileError;
use mchprs_simulation::block_changes::BlockChanges;
use mchprs_simulation::readback::Readback;
use mchprs_simulation::ticks::SimulationError;
use mchprs_simulation::{Blocks, Command, Event, Update, View};
use rustc_hash::FxHashSet;
use std::collections::HashSet;
use std::mem;
use std::sync::Arc;
use std::time::Instant;
use tracing::{debug, error, trace};

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum StartError {
    #[error("Redpiler is already running.")]
    Running,
    #[error("Redpiler didn't start because the plot couldn't be saved.")]
    Unsaved,
    #[error("Redpiler didn't start because its compile thread couldn't start.")]
    CompileThread,
}

impl Plot {
    pub(super) fn poll_compile(&mut self) {
        let PlotSimulation::Compiling(compiling) = &mut self.simulation else {
            return;
        };
        let progressed = compiling.job.progressed();
        let step = compiling.job.step();
        let finished = compiling.job.finished();
        if progressed {
            trace!("Redpiler compile step {step:?}");
            self.refresh_scoreboard();
        }
        match finished {
            None => {}
            Some(Ok(compilation)) => self.run_redpiler(compilation),
            Some(Err(error)) => self.fail_compile(&error),
        }
    }

    pub(super) fn poll_simulation(&mut self) {
        let PlotSimulation::Compiled(compiled) = &mut self.simulation else {
            return;
        };
        match compiled.simulation.poll() {
            Ok(Some(update)) => self.apply_update(update),
            Ok(None) => {}
            Err(error) => self.fail_redpiler(error),
        }
    }

    pub fn sync_redpiler(&mut self) {
        let PlotSimulation::Compiled(compiled) = &mut self.simulation else {
            return;
        };
        compiled.simulation.send(Command::Show(View::All));
        loop {
            let PlotSimulation::Compiled(compiled) = &mut self.simulation else {
                return;
            };
            match compiled.simulation.wait() {
                Ok(update) => {
                    let shown = update.events.contains(&Event::Shown(View::All));
                    self.apply_update(update);
                    if shown {
                        return;
                    }
                }
                Err(error) => return self.fail_redpiler(error),
            }
        }
    }

    pub fn start_redpiler(&mut self, setup: Setup) -> Result<(), StartError> {
        self.save();
        if !self.simulation.is_editable() {
            return Err(StartError::Running);
        }
        if !self.world.save_state.is_saved() {
            return Err(StartError::Unsaved);
        }
        debug!("Starting redpiler with {setup}");
        let blocks = Arc::new(self.world.copy_blocks());
        let observed = self
            .probes
            .iter()
            .flat_map(|probe| probe.positions().iter().copied())
            .collect();
        let job = CompileJob::spawn(
            Arc::clone(&blocks),
            self.world.region(),
            self.world.redstone.ticks.entries().to_vec(),
            observed,
            setup.clone(),
            format!("p{},{}", self.world.x, self.world.z),
        )
        .map_err(|error| {
            error!("Redpiler couldn't start its compile thread: {error}");
            StartError::CompileThread
        })?;
        self.advances.abandon(self.ticks);
        self.halted = false;
        let ticks = mem::take(&mut self.world.redstone.ticks).into_entries();
        self.simulation = PlotSimulation::Compiling(Compiling {
            setup,
            job,
            before: Before { blocks, ticks },
            started: Instant::now(),
        });
        self.refresh_scoreboard();
        Ok(())
    }

    pub fn stop_redpiler(&mut self, restore: bool) {
        self.poll_simulation();
        match self.take_simulation() {
            PlotSimulation::World(schedule) => {
                self.simulation = PlotSimulation::World(schedule);
                return;
            }
            PlotSimulation::Compiling(compiling) => {
                debug!("Cancelling the redpiler compile");
                self.world.redstone.ticks = TickQueue::new(compiling.before.ticks);
                self.run_world();
            }
            PlotSimulation::Compiled(compiled) => self.stop_compiled(compiled, restore),
        }
        self.publish_world();
    }

    pub(in crate::plot) fn use_block_in_circuit(&mut self, pos: BlockPos) -> bool {
        let PlotSimulation::Compiled(compiled) = &mut self.simulation else {
            return false;
        };
        let Some(input) = Input::of_use(self.world.block(pos)) else {
            return false;
        };
        if !compiled.takes_input_at(pos) {
            return false;
        }
        compiled.simulation.send(Command::Drive(pos, input));
        true
    }

    pub(in crate::plot) fn set_entity_boxes(&mut self, boxes: Vec<Aabb>) {
        let PlotSimulation::Compiled(compiled) = &mut self.simulation else {
            self.world.entity_boxes = boxes;
            return;
        };
        let plates: HashSet<BlockPos> = self
            .world
            .entity_boxes
            .iter()
            .chain(&boxes)
            .flat_map(Aabb::block_positions)
            .filter(|&pos| self.world.block(pos).pressure_plate().is_some())
            .collect();
        let was_touched: Vec<(BlockPos, bool)> = plates
            .into_iter()
            .map(|pos| (pos, plate::is_touched(&self.world, pos)))
            .collect();
        self.world.entity_boxes = boxes;
        for (pos, was_touched) in was_touched {
            let touched = plate::is_touched(&self.world, pos);
            if touched != was_touched && compiled.takes_input_at(pos) {
                compiled
                    .simulation
                    .send(Command::Drive(pos, Input::Touch(touched)));
            }
        }
    }

    fn stop_compiled(&mut self, compiled: Compiled, restore: bool) {
        debug!("Stopping redpiler");
        let backend = compiled.setup.name().to_owned();
        if !restore && !compiled.capabilities.write_back {
            self.broadcast(&format!(
                "The {backend} backend doesn't keep the state it reached, so the plot returns to its state before compiling."
            ));
        }
        let Compiled {
            simulation,
            before,
            mut written,
            ..
        } = compiled;
        let end = match simulation.stop() {
            Ok(end) => end,
            Err(error) => return self.fail_redpiler_with(before, error),
        };
        extend_written(&mut written, &end.changes);
        end.changes.apply(&mut self.world);
        self.advances.abandon(self.ticks);
        let had_ticks = !before.ticks.is_empty();
        match (end.state, restore) {
            (Readback::Complete(write_back), false) => write_back.apply(&mut self.world),
            (Readback::Complete(_), true) | (Readback::Incomplete, _) => {
                self.restore(&written, before);
            }
        }
        if had_ticks || !self.world.redstone.ticks.entries().is_empty() {
            self.world.mark_unsaved();
        }
        self.run_world();
    }

    fn restore(&mut self, written: &FxHashSet<BlockPos>, before: Before) {
        for &pos in written {
            self.world.set_block(pos, before.blocks.block(pos));
            let block_entity = before.blocks.block_entity(pos);
            if self.world.block_entity(pos) != block_entity {
                self.world.set_block_entity(pos, block_entity.cloned());
            }
        }
        self.world.redstone.ticks = TickQueue::new(before.ticks);
    }

    fn apply_update(&mut self, update: Update<Blocks>) {
        let PlotSimulation::Compiled(compiled) = &mut self.simulation else {
            return;
        };
        extend_written(&mut compiled.written, &update.changes);
        let ran = update.ticks.saturating_sub(compiled.ticks);
        compiled.ticks = update.ticks;
        compiled.behind = update.behind;
        if update.events.contains(&Event::Shown(View::Observed)) {
            compiled.observing = false;
        }
        let start_tick = compiled.start_tick;
        update.changes.apply(&mut self.world);
        self.record_ticks(ran);
        self.handle_events(&update.events, start_tick);
        self.show_world();
    }

    fn run_redpiler(&mut self, Compilation { simulation, layout }: Compilation) {
        let compiling = match self.take_simulation() {
            PlotSimulation::Compiling(compiling) => compiling,
            other => {
                self.simulation = other;
                return;
            }
        };
        let backend = compiling.setup.name().to_owned();
        let message = format!(
            "Redpiler compiled the plot for the {backend} backend in {:?}.",
            compiling.started.elapsed()
        );
        let mut compiled = Compiled {
            setup: compiling.setup,
            layout,
            capabilities: simulation.capabilities(),
            simulation,
            before: compiling.before,
            written: FxHashSet::default(),
            start_tick: self.ticks,
            ticks: 0,
            behind: false,
            observing: false,
        };
        for pos in compiled.layout.positions() {
            let Some(pressure_plate) = self.world.block(pos).pressure_plate() else {
                continue;
            };
            let touched = plate::is_touched(&self.world, pos);
            if (pressure_plate.powered || touched) && compiled.takes_input_at(pos) {
                compiled
                    .simulation
                    .send(Command::Drive(pos, Input::Touch(touched)));
            }
        }
        let halts = compiled.capabilities.breakpoints;
        self.simulation = PlotSimulation::Compiled(compiled);
        self.restart_watch();
        self.update_tick_rate();
        debug!("{message}");
        self.broadcast(&message);
        let watched = self.probes.iter().any(|probe| probe.breakpoint.is_some());
        if watched && !halts {
            self.broadcast(&format!(
                "The {backend} backend can't watch probes, so breakpoints wait until Redpiler stops."
            ));
        }
        self.refresh_scoreboard();
    }

    fn fail_compile(&mut self, error: &anyhow::Error) {
        let compiling = match self.take_simulation() {
            PlotSimulation::Compiling(compiling) => compiling,
            other => {
                self.simulation = other;
                return;
            }
        };
        error!("Redpiler failed to compile: {error}");
        self.world.redstone.ticks = TickQueue::new(compiling.before.ticks);
        let advice = match error.downcast_ref() {
            Some(CompileError::Stale(_)) => " Run //update -p to bring every block up to date.",
            _ => "",
        };
        self.broadcast(&format!(
            "Redpiler failed and was stopped: {error}.{advice}"
        ));
        self.run_world();
    }

    fn fail_redpiler(&mut self, error: SimulationError) {
        match self.take_simulation() {
            PlotSimulation::Compiled(compiled) => self.fail_redpiler_with(compiled.before, error),
            other => self.simulation = other,
        }
    }

    fn fail_redpiler_with(&mut self, before: Before, error: SimulationError) {
        error!("Redpiler failed: {error}");
        self.advances.abandon(self.ticks);
        let blocks = Arc::unwrap_or_clone(before.blocks);
        self.world.chunks = blocks.chunks;
        self.world.redstone.ticks = TickQueue::new(before.ticks);
        self.world.pending_block_entities.clear();
        self.world.mark_unsaved();
        self.recheck_pressure_plates();
        for player in 0..self.players.len() {
            self.update_view_pos_for_player(player, true);
        }
        self.broadcast(&format!(
            "Redpiler failed and was stopped: {error}. The plot was restored to its state before compiling."
        ));
        self.run_world();
    }
}

fn extend_written(written: &mut FxHashSet<BlockPos>, changes: &BlockChanges) {
    written.extend(changes.blocks.iter().map(|&(pos, _)| pos));
    written.extend(changes.block_entities.iter().map(|&(pos, _)| pos));
}
