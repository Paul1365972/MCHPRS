use crate::{Command, Event, Goal, SendRate, SendSchedule, TickRate, TickReport, TickSchedule};
use mchprs_blocks::BlockPos;
use mchprs_redpiler::compile::Stage;
use mchprs_redpiler::engine::{DriveError, EngineError, Input};
use mchprs_redpiler::simulation::Simulation;
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
    let Some((simulation, deferred)) = build_simulation(build, &commands, &mut events) else {
        return;
    };
    events.send(Event::Started(simulation.circuit().clone()));
    let now = Instant::now();
    let mut runner = Runner {
        simulation,
        events,
        ticks: TickSchedule::new(tick_rate, now),
        sends: SendSchedule::new(send_rate, now),
        advance: None,
        unsent_ticks: 0,
        tally: Tally::new(now),
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

struct Tally {
    ticks: u64,
    ticking: Duration,
    since: Instant,
}

impl Tally {
    fn new(now: Instant) -> Tally {
        Tally {
            ticks: 0,
            ticking: Duration::ZERO,
            since: now,
        }
    }
}

struct Runner {
    simulation: Simulation,
    events: Events,
    ticks: TickSchedule,
    sends: SendSchedule,
    advance: Option<Advance>,
    unsent_ticks: u64,
    tally: Tally,
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
            let received = match self.next_deadline() {
                Some(deadline) => {
                    match commands.recv_timeout(deadline.saturating_duration_since(Instant::now()))
                    {
                        Ok(command) => Some(command),
                        Err(RecvTimeoutError::Timeout) => None,
                        Err(RecvTimeoutError::Disconnected) => return Ok(()),
                    }
                }
                None => match commands.recv() {
                    Ok(command) => Some(command),
                    Err(_) => return Ok(()),
                },
            };
            if let Some(command) = received {
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
            Command::SetTickRate(rate) => {
                let now = Instant::now();
                if (rate == TickRate::Paused) != (self.ticks.rate() == TickRate::Paused) {
                    self.tally = Tally::new(now);
                }
                self.ticks.set_rate(rate, now);
            }
            Command::SetSendRate(rate) => self.sends.set_rate(rate, Instant::now()),
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
                self.events.send(Event::Synced(changes));
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
        if self.advance.is_some() || self.publish_now {
            return Some(Instant::now());
        }
        let send = self.sends.next_send().filter(|_| self.unsent_ticks > 0);
        match (self.ticks.next_tick(), send) {
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
        let due = self.ticks.due(Instant::now());
        if due > 0 {
            let ran = self.run_batch(due, false)?;
            self.ticks.ran(ran.ticks);
        }
        Ok(())
    }

    fn finish_advance(&mut self, settled: bool) {
        let Some(advance) = self.advance.take() else {
            return;
        };
        if advance.ran > 0 {
            self.ticks.skip_overdue(Instant::now());
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
        self.unsent_ticks += ran;
        Ok(Ran {
            ticks: ran,
            settled,
        })
    }

    fn publish_if_due(&mut self) -> Result<(), EngineError> {
        let due = self.unsent_ticks > 0 && self.sends.is_due(Instant::now());
        if self.publish_now || due {
            self.publish()?;
        }
        Ok(())
    }

    fn publish(&mut self) -> Result<(), EngineError> {
        self.publish_now = false;
        let changes = self.simulation.sync()?;
        self.unsent_ticks = 0;
        if !changes.is_empty() {
            self.events.send(Event::Batch(changes));
        }
        self.sends.sent(Instant::now());
        Ok(())
    }

    fn report_ticks(&mut self) {
        let now = Instant::now();
        self.events.send(Event::Ticked(TickReport {
            ticks: self.tally.ticks,
            ticking: self.tally.ticking,
            period: now - self.tally.since,
            advanced: self.advance.as_ref().map_or(0, |advance| advance.ran),
            behind: self.advance.is_none() && self.ticks.is_behind(),
        }));
        self.tally = Tally::new(now);
    }
}
