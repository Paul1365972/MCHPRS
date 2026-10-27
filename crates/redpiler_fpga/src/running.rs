use crate::engine::FpgaEngine;
use crate::protocol::{Mode, MAX_COUNT};
use mchprs_redpiler::circuit::NodeId;
use mchprs_redpiler::nodes::Nodes;
use mchprs_redstone::tick_queue::TickPriority;
use mchprs_simulation::handle::{self, Request, Updates};
use mchprs_simulation::input::DriveError;
use mchprs_simulation::ticks::SimulationError;
use mchprs_simulation::{Command, End, Event, Run, Running, Simulation, Update, View};
use std::mem;
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender};
use std::time::{Duration, Instant};

const SETTLE_CHECK: Duration = Duration::from_millis(10);

type Reply = Sender<Result<End<Nodes>, SimulationError>>;

#[derive(Clone, Copy)]
enum Goal {
    Ticks { start: u64, ticks: u64 },
    Settled,
}

struct Worker {
    engine: FpgaEngine,
    goal: Option<Goal>,
    next_check: Option<Instant>,
    is_behind_schedule: bool,
    pending: Update<Nodes>,
    updates: Updates<Nodes>,
}

pub fn start(engine: FpgaEngine) -> Result<Box<dyn Running<Nodes>>, SimulationError> {
    let capabilities = engine.capabilities();
    handle::spawn("fpga", capabilities, move |requests, updates| {
        let mut worker = Worker {
            engine,
            goal: None,
            next_check: None,
            is_behind_schedule: false,
            pending: Update::default(),
            updates,
        };
        match worker.serve(&requests) {
            Ok(Some(reply)) => {
                let unshown = mem::take(&mut worker.pending.changes);
                let end = Box::new(worker.engine).stop().map(|mut end| {
                    let mut changes = unshown;
                    changes.extend([end.changes]);
                    end.changes = changes;
                    end
                });
                let _ = reply.send(end);
                Ok(())
            }
            Ok(None) => Ok(()),
            Err(error) => {
                let _ = worker.engine.set_mode(Mode::Paused);
                Err(error)
            }
        }
    })
}

impl Worker {
    fn serve(
        &mut self,
        requests: &Receiver<Request<Nodes>>,
    ) -> Result<Option<Reply>, SimulationError> {
        loop {
            if self.next_check.is_some_and(|check| check <= Instant::now()) {
                self.check_goal()?;
                self.deliver_if_eventful();
            }
            let request = match self.next_check {
                None => requests.recv().ok(),
                Some(check) => {
                    match requests.recv_timeout(check.saturating_duration_since(Instant::now())) {
                        Ok(request) => Some(request),
                        Err(RecvTimeoutError::Timeout) => continue,
                        Err(RecvTimeoutError::Disconnected) => None,
                    }
                }
            };
            match request {
                Some(Request::Command(command)) => self.handle(command)?,
                Some(Request::Stop(reply)) => {
                    self.engine.set_mode(Mode::Paused)?;
                    return Ok(Some(reply));
                }
                None => {
                    self.engine.set_mode(Mode::Paused)?;
                    return Ok(None);
                }
            }
            self.deliver_if_eventful();
        }
    }

    fn handle(&mut self, command: Command<NodeId>) -> Result<(), SimulationError> {
        match command {
            Command::Pause => {
                self.set_mode(Mode::Paused)?;
                self.replace_goal();
            }
            Command::Run(run) => self.run(run)?,
            Command::Drive(node, input) => match self.engine.drive(node, input) {
                Ok(()) => {
                    self.record_changes();
                    self.finish_counted_run();
                    self.deliver();
                }
                Err(DriveError::Refused(error)) => {
                    self.pending.events.push(Event::Refused(node, error));
                }
                Err(DriveError::Simulation(error)) => return Err(error),
            },
            Command::Watch(_) => {}
            Command::Show(view) => {
                self.show(view)?;
                self.finish_counted_run();
                self.pending.events.push(Event::Shown(view));
            }
        }
        Ok(())
    }

    fn run(&mut self, run: Run) -> Result<(), SimulationError> {
        match run {
            Run::Every(interval) => {
                let clocks = interval.as_secs_f64() * self.engine.clock_hertz();
                let clocks_per_tick = (clocks.round() as u64).clamp(1, MAX_COUNT);
                self.set_mode(Mode::Periodic { clocks_per_tick })?;
                self.replace_goal();
            }
            Run::Unlimited => {
                self.set_mode(Mode::Unlimited)?;
                self.replace_goal();
            }
            Run::Ticks(ticks) => {
                self.set_mode(Mode::counted(ticks)?)?;
                self.replace_goal();
                let start = self.engine.ticks();
                self.goal = Some(Goal::Ticks { start, ticks });
                self.finish_counted_run();
            }
            Run::UntilSettled => {
                self.set_mode(Mode::Paused)?;
                self.replace_goal();
                self.goal = Some(Goal::Settled);
                self.check_goal()?;
                if self.goal.is_some() {
                    self.set_mode(Mode::Unlimited)?;
                }
            }
        }
        Ok(())
    }

    fn set_mode(&mut self, mode: Mode) -> Result<(), SimulationError> {
        self.engine.set_mode(mode)?;
        let tick_clocks = TickPriority::ALL.len() as u64;
        self.is_behind_schedule =
            matches!(mode, Mode::Periodic { clocks_per_tick } if clocks_per_tick < tick_clocks);
        Ok(())
    }

    fn show(&mut self, view: View) -> Result<(), SimulationError> {
        self.engine.show(view, &mut self.pending.changes)
    }

    fn record_changes(&mut self) {
        self.pending.changes.extend([self.engine.take_changes()]);
    }

    fn has_counted(&self, goal: Goal) -> bool {
        match goal {
            Goal::Ticks { start, ticks } => self.engine.ticks().wrapping_sub(start) >= ticks,
            Goal::Settled => false,
        }
    }

    fn finish_counted_run(&mut self) {
        let Some(goal @ Goal::Ticks { .. }) = self.goal else {
            return;
        };
        if self.has_counted(goal) {
            self.goal = None;
            self.pending.events.push(Event::RunFinished);
        }
        self.plan_check();
    }

    fn replace_goal(&mut self) {
        match self.goal.take() {
            Some(goal @ Goal::Ticks { .. }) if self.has_counted(goal) => {
                self.pending.events.push(Event::RunFinished);
            }
            Some(_) => self.pending.events.push(Event::RunInterrupted),
            None => {}
        }
        self.plan_check();
    }

    fn plan_check(&mut self) {
        let delay = match self.goal {
            Some(Goal::Ticks { start, ticks }) => {
                let left = ticks.saturating_sub(self.engine.ticks().wrapping_sub(start));
                Some(self.engine.run_time(left))
            }
            Some(Goal::Settled) => Some(SETTLE_CHECK),
            None => None,
        };
        self.next_check = delay.and_then(|delay| Instant::now().checked_add(delay));
    }

    fn check_goal(&mut self) -> Result<(), SimulationError> {
        match self.goal {
            Some(Goal::Ticks { .. }) => {
                self.show(View::Observed)?;
                self.finish_counted_run();
            }
            Some(Goal::Settled) => {
                let snapshot = self.engine.read_snapshot()?;
                self.record_changes();
                if snapshot.pending_ticks.is_empty() {
                    self.set_mode(Mode::Paused)?;
                    self.goal = None;
                    self.pending.events.push(Event::RunFinished);
                }
                self.plan_check();
            }
            None => {}
        }
        Ok(())
    }

    fn deliver_if_eventful(&mut self) {
        if !self.pending.events.is_empty() {
            self.deliver();
        }
    }

    fn deliver(&mut self) {
        let mut update = mem::take(&mut self.pending);
        update.ticks = self.engine.ticks();
        update.behind = self.is_behind_schedule;
        let _ = self.updates.send(Ok(update));
    }
}
