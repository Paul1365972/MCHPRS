mod compiled;
mod world;

use super::compile::CompileJob;
use super::scoreboard::RedpilerStatus;
use super::world::PlotBlocks;
use super::Plot;
use mchprs_backends::Setup;
use mchprs_blocks::BlockPos;
use mchprs_redstone::tick_queue::TickEntry;
use mchprs_save_data::plot_data::Tps;
use mchprs_simulation::layout::Layout;
use mchprs_simulation::ports::Role;
use mchprs_simulation::schedule::Schedule;
use mchprs_simulation::{Blocks, Capabilities, Command, Event, Run, Running, View};
use rustc_hash::FxHashSet;
use std::mem;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tracing::warn;

pub enum PlotSimulation {
    World(Schedule),
    Compiling(Compiling),
    Compiled(Compiled),
}

impl PlotSimulation {
    pub fn is_editable(&self) -> bool {
        matches!(self, PlotSimulation::World(_))
    }

    pub fn is_compiling(&self) -> bool {
        matches!(self, PlotSimulation::Compiling(_))
    }

    pub(super) fn next_tick(&self) -> Option<Instant> {
        match self {
            PlotSimulation::World(schedule) => schedule.next_tick(),
            PlotSimulation::Compiling(_) | PlotSimulation::Compiled(_) => None,
        }
    }

    pub(super) fn unsettling_backend(&self) -> Option<&str> {
        match self {
            PlotSimulation::Compiled(compiled) if !compiled.capabilities.settling => {
                Some(compiled.setup.name())
            }
            _ => None,
        }
    }

    pub(super) fn status(&self) -> (RedpilerStatus, bool) {
        match self {
            PlotSimulation::World(schedule) => (RedpilerStatus::Stopped, schedule.is_behind()),
            PlotSimulation::Compiling(compiling) => (
                RedpilerStatus::Compiling {
                    setup: compiling.setup.clone(),
                    step: compiling.job.step(),
                },
                false,
            ),
            PlotSimulation::Compiled(compiled) => (
                RedpilerStatus::Running {
                    setup: compiled.setup.clone(),
                },
                compiled.behind,
            ),
        }
    }
}

pub struct Compiling {
    setup: Setup,
    job: CompileJob,
    before: Before,
    started: Instant,
}

pub struct Compiled {
    setup: Setup,
    layout: Layout,
    simulation: Box<dyn Running<Blocks>>,
    capabilities: Capabilities,
    before: Before,
    written: FxHashSet<BlockPos>,
    ticks: u64,
    behind: bool,
    observing: bool,
}

impl Compiled {
    fn takes_input_at(&self, pos: BlockPos) -> bool {
        let component = self.layout.at(pos);
        component.is_some_and(|component| component.port == Some(Role::Input))
    }

    fn observe(&mut self) {
        if !self.observing {
            self.simulation.send(Command::Show(View::Observed));
            self.observing = true;
        }
    }
}

struct Before {
    blocks: Arc<PlotBlocks>,
    ticks: Vec<TickEntry>,
}

impl Plot {
    fn target_run(&self) -> Option<Run> {
        if self.players.is_empty() {
            return None;
        }
        match self.tps {
            Tps::Unlimited => Some(Run::Unlimited),
            Tps::Limited(tps) => match Duration::try_from_secs_f64(1.0 / f64::from(tps)) {
                Err(_) => None,
                Ok(interval) if interval.is_zero() => Some(Run::Unlimited),
                Ok(interval) => Some(Run::Every(interval)),
            },
        }
    }

    pub(super) fn update_tick_rate(&mut self) {
        self.send_run(self.target_run());
    }

    pub(super) fn send_run(&mut self, run: Option<Run>) {
        let PlotSimulation::World(schedule) = &mut self.simulation else {
            if let PlotSimulation::Compiled(compiled) = &mut self.simulation {
                let command = run.map_or(Command::Pause, Command::Run);
                compiled.simulation.send(command);
            }
            return;
        };
        let mut events = Vec::new();
        match run {
            Some(run) => schedule.set(run, Instant::now(), &mut events),
            None => schedule.pause(&mut events),
        }
        self.handle_events(&events);
    }

    pub(super) fn update_simulation(&mut self) {
        match self.simulation {
            PlotSimulation::World(_) => self.run_world_ticks(),
            PlotSimulation::Compiling(_) => self.poll_compile(),
            PlotSimulation::Compiled(_) => self.poll_simulation(),
        }
    }

    pub(super) fn send_world(&mut self) {
        let PlotSimulation::Compiled(compiled) = &mut self.simulation else {
            return self.publish_world();
        };
        compiled.observe();
        self.send_schedule.sent(Instant::now());
    }

    fn take_simulation(&mut self) -> PlotSimulation {
        let paused = PlotSimulation::World(Schedule::default());
        mem::replace(&mut self.simulation, paused)
    }

    fn record_ticks(&mut self, ticks: u64) {
        self.ticks += ticks;
        self.tick_history.record(ticks);
    }

    fn handle_events(&mut self, events: &[Event<BlockPos>]) {
        for &event in events {
            match event {
                Event::RunFinished => self.advances.end(self.ticks, true),
                Event::RunInterrupted => self.advances.end(self.ticks, false),
                Event::Refused(pos, error) => {
                    warn!("Redpiler refused a drive: the block at {pos} {error}");
                }
                Event::Shown(_) => {}
            }
        }
    }
}
