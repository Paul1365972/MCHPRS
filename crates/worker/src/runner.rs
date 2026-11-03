use crate::pacing::Pacing;
use crate::{Command, Event, Goal, SendRate, TickRate, TickReport};
use mchprs_blocks::BlockPos;
use mchprs_redpiler::compile::Stage;
use mchprs_redpiler::engine::{DriveError, EngineError, Input};
use mchprs_redpiler::simulation::Simulation;
use std::mem;
use std::ops::ControlFlow;
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender, TryRecvError};
use std::time::{Duration, Instant};

const BATCH_TIME: Duration = Duration::from_millis(10);

pub(super) fn run<F>(
    build: F,
    tick_rate: TickRate,
    send_rate: SendRate,
    commands: Receiver<Command>,
    event_sender: Sender<Event>,
) where
    F: FnOnce(&mut dyn FnMut(Stage) -> ControlFlow<()>) -> Result<Simulation, EngineError>,
{
    let mut events = Events(event_sender);
    let mut pacing = Pacing::new(tick_rate, send_rate);
    let Some((simulation, deferred)) = build_simulation(build, &commands, &mut events, &mut pacing)
    else {
        return;
    };
    events.send(Event::Started(simulation.circuit().clone()));
    pacing.forget_backlog();
    let mut runner = Runner {
        simulation,
        events,
        pacing,
        advance: None,
        tally: Tally::default(),
        ticks_since_batch: 0,
        publish_now: false,
    };
    if let Err(error) = runner.run(deferred, &commands) {
        runner.finish_advance(false);
        runner.events.send(Event::Failed(error));
    }
}

struct Events(Sender<Event>);

impl Events {
    fn send(&mut self, event: Event) {
        // A worker whose handle is gone has no one to tell
        let _ = self.0.send(event);
    }
}

fn build_simulation<F>(
    build: F,
    commands: &Receiver<Command>,
    events: &mut Events,
    pacing: &mut Pacing,
) -> Option<(Simulation, Vec<Command>)>
where
    F: FnOnce(&mut dyn FnMut(Stage) -> ControlFlow<()>) -> Result<Simulation, EngineError>,
{
    let mut stopped = false;
    let mut deferred = Vec::new();
    let result = build(&mut |stage| {
        events.send(Event::Progress(stage));
        loop {
            match commands.try_recv() {
                Ok(Command::Stop { .. }) | Err(TryRecvError::Disconnected) => {
                    stopped = true;
                    return ControlFlow::Break(());
                }
                Ok(Command::SetTickRate(rate)) => pacing.set_tick_rate(rate),
                Ok(Command::SetSendRate(rate)) => pacing.set_send_rate(rate),
                Ok(command) => deferred.push(command),
                Err(TryRecvError::Empty) => return ControlFlow::Continue(()),
            }
        }
    });
    if stopped {
        events.send(Event::Cancelled);
        return None;
    }
    match result {
        Ok(simulation) => Some((simulation, deferred)),
        Err(error) => {
            events.send(Event::Failed(error));
            None
        }
    }
}

struct Advance {
    goal: Goal,
    ran: u64,
}

struct Ran {
    ticks: u64,
    settled: bool,
}

#[derive(Default)]
struct Tally {
    ticks: u64,
    ticking: Duration,
}

struct Runner {
    simulation: Simulation,
    events: Events,
    pacing: Pacing,
    advance: Option<Advance>,
    tally: Tally,
    ticks_since_batch: u64,
    publish_now: bool,
}

impl Runner {
    fn run(
        &mut self,
        deferred: Vec<Command>,
        commands: &Receiver<Command>,
    ) -> Result<(), EngineError> {
        for command in deferred {
            if self.handle(command)?.is_break() {
                return Ok(());
            }
        }
        loop {
            let busy = self.pacing.tick_rate == TickRate::Unlimited
                || self.advance.is_some()
                || self.publish_now;
            let command = if busy {
                match commands.try_recv() {
                    Ok(command) => Some(command),
                    Err(TryRecvError::Empty) => None,
                    Err(TryRecvError::Disconnected) => return Ok(()),
                }
            } else {
                match self.next_deadline() {
                    Some(deadline) => {
                        let timeout = deadline.saturating_duration_since(Instant::now());
                        match commands.recv_timeout(timeout) {
                            Ok(command) => Some(command),
                            Err(RecvTimeoutError::Timeout) => None,
                            Err(RecvTimeoutError::Disconnected) => return Ok(()),
                        }
                    }
                    None => match commands.recv() {
                        Ok(command) => Some(command),
                        Err(_) => return Ok(()),
                    },
                }
            };
            if let Some(command) = command {
                if self.handle(command)?.is_break() {
                    return Ok(());
                }
                continue;
            }
            self.run_ticks()?;
            self.publish_if_due()?;
        }
    }

    fn handle(&mut self, command: Command) -> Result<ControlFlow<()>, EngineError> {
        match command {
            Command::SetTickRate(rate) => self.pacing.set_tick_rate(rate),
            Command::SetSendRate(rate) => self.pacing.set_send_rate(rate),
            Command::Drive { position, input } => self.drive(position, input)?,
            Command::Advance(goal) => {
                self.finish_advance(false);
                self.advance = Some(Advance { goal, ran: 0 });
            }
            Command::StopAdvancing => {
                if self.advance.is_some() {
                    self.publish()?;
                    self.finish_advance(false);
                }
            }
            Command::ReportTicks => self.report_ticks(),
            Command::Sync => {
                let changes = self.simulation.sync_all()?;
                let ticks = self.simulation.pending_ticks()?;
                self.events.send(Event::Synced { changes, ticks });
            }
            Command::Stop { restore } => {
                self.finish_advance(false);
                self.report_ticks();
                if restore {
                    self.simulation.reset()?;
                }
                let changes = self.simulation.sync_all()?;
                let ticks = self.simulation.pending_ticks()?;
                self.events.send(Event::Stopped { changes, ticks });
                return Ok(ControlFlow::Break(()));
            }
        }
        Ok(ControlFlow::Continue(()))
    }

    fn report_ticks(&mut self) {
        let tally = mem::take(&mut self.tally);
        self.events.send(Event::Ticked(TickReport {
            ticks: tally.ticks,
            ticking: tally.ticking,
            advanced: self.advance.as_ref().map_or(0, |advance| advance.ran),
        }));
    }

    fn drive(&mut self, position: BlockPos, input: Input) -> Result<(), EngineError> {
        match self.simulation.drive(position, input) {
            Ok(()) => self.publish_now = true,
            Err(DriveError::Engine(error)) => return Err(error),
            Err(error) => self.events.send(Event::Rejected { position, error }),
        }
        Ok(())
    }

    /// Sending is only due once ticks have run, so an idle worker waits for commands.
    fn next_deadline(&self) -> Option<Instant> {
        let tick = self.pacing.tick_interval().map(|_| self.pacing.next_tick);
        let send = self
            .pacing
            .send_interval
            .filter(|_| self.ticks_since_batch > 0)
            .map(|_| self.pacing.next_send);
        match (tick, send) {
            (Some(tick), Some(send)) => Some(tick.min(send)),
            (tick, send) => tick.or(send),
        }
    }

    fn run_ticks(&mut self) -> Result<(), EngineError> {
        if let Some(advance) = &self.advance {
            let (limit, until_settled) = match advance.goal {
                Goal::Ticks(ticks) => (ticks - advance.ran, false),
                Goal::Settled => (u64::MAX, true),
            };
            let ran = self.run_batch(limit, until_settled)?;
            let advance = self.advance.as_mut().expect("an advance is running");
            advance.ran += ran.ticks;
            if ran.settled || ran.ticks == limit {
                self.publish()?;
                self.finish_advance(ran.settled);
            }
            return Ok(());
        }
        let ticks = self.pacing.due_ticks(Instant::now());
        if ticks > 0 {
            let ran = self.run_batch(ticks, false)?;
            self.pacing.ran_ticks(ran.ticks);
        }
        Ok(())
    }

    fn finish_advance(&mut self, settled: bool) {
        let Some(advance) = self.advance.take() else {
            return;
        };
        if advance.ran > 0 {
            self.pacing.skip_overdue(Instant::now());
        }
        self.events.send(Event::Advanced {
            ran: advance.ran,
            settled,
        });
    }

    fn run_batch(&mut self, limit: u64, until_settled: bool) -> Result<Ran, EngineError> {
        let start = Instant::now();
        let mut ran = 0;
        let mut settled = false;
        let mut chunk = 1;
        while ran < limit {
            let ticks = chunk.min(limit - ran);
            if until_settled {
                if let Some(taken) = self.simulation.settle(ticks)? {
                    ran += taken;
                    settled = true;
                    break;
                }
            } else {
                self.simulation.run(ticks)?;
            }
            ran += ticks;
            let elapsed = start.elapsed();
            if elapsed >= BATCH_TIME {
                break;
            }
            let rate = ran as f64 / elapsed.as_secs_f64();
            let fitting = (rate * (BATCH_TIME - elapsed).as_secs_f64()) as u64;
            // Doubling at most, so ticks that suddenly cost more can't overshoot the batch by much
            chunk = chunk.saturating_mul(2).min(fitting).max(1);
        }
        self.tally.ticks += ran;
        self.tally.ticking += start.elapsed();
        self.ticks_since_batch += ran;
        Ok(Ran {
            ticks: ran,
            settled,
        })
    }

    fn publish_if_due(&mut self) -> Result<(), EngineError> {
        let due = self.ticks_since_batch > 0
            && self
                .pacing
                .send_interval
                .is_some_and(|_| Instant::now() >= self.pacing.next_send);
        if self.publish_now || due {
            self.publish()?;
        }
        Ok(())
    }

    fn publish(&mut self) -> Result<(), EngineError> {
        self.publish_now = false;
        let changes = self.simulation.sync()?;
        self.ticks_since_batch = 0;
        if !changes.is_empty() {
            self.events.send(Event::Batch(changes));
        }
        if let Some(interval) = self.pacing.send_interval {
            self.pacing.next_send = Instant::now() + interval;
        }
        Ok(())
    }
}
