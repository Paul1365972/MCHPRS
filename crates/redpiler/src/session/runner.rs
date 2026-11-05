use super::{Command, Event, TickReport};
use crate::compile::BuildContext;
use crate::engine::{DriveError, EngineError, Input};
use crate::simulation::Simulation;
use mchprs_blocks::BlockPos;
use mchprs_redstone::pace::{Pacer, SendRate, SendSchedule, TickRate};
use std::ops::ControlFlow;
use std::sync::atomic::{AtomicBool, Ordering};
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
    F: FnOnce(&mut BuildContext<'_>) -> Result<Simulation, EngineError>,
{
    let mut events = Events(event_sender);
    let Some((simulation, deferred)) = build_simulation(build, &commands, &mut events) else {
        return;
    };
    events.send(Event::Started(simulation.circuit().clone()));
    let origin = Instant::now();
    let mut runner = Runner {
        simulation,
        events,
        origin,
        pacer: Pacer::new(tick_rate, Duration::ZERO),
        sends: SendSchedule::new(send_rate, Duration::ZERO),
        unsent_ticks: 0,
        tally: Tally::new(origin),
        publish_now: false,
    };
    if let Err(error) = runner.run(deferred, &commands) {
        runner.end_advance();
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
    F: FnOnce(&mut BuildContext<'_>) -> Result<Simulation, EngineError>,
{
    let cancelled = AtomicBool::new(false);
    let mut deferred = Vec::new();
    let mut progress = |stage| {
        events.send(Event::Progress(stage));
        loop {
            match commands.try_recv() {
                Ok(Command::Stop { .. }) | Err(TryRecvError::Disconnected) => {
                    cancelled.store(true, Ordering::Relaxed);
                    return;
                }
                Ok(command) => deferred.push(command),
                Err(TryRecvError::Empty) => return,
            }
        }
    };
    let result = build(&mut BuildContext::new(&mut progress, &cancelled));
    if cancelled.load(Ordering::Relaxed) {
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
    origin: Instant,
    pacer: Pacer,
    sends: SendSchedule,
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
                if (rate == TickRate::Paused) != (self.pacer.rate() == TickRate::Paused) {
                    self.tally = Tally::new(Instant::now());
                }
                self.pacer.set_rate(rate, self.now());
            }
            Command::SetSendRate(rate) => self.sends.set_rate(rate, self.now()),
            Command::Drive { position, input } => self.drive(position, input)?,
            Command::Advance(goal) => {
                if let Some(advanced) = self.pacer.advance(Some(goal), self.now()) {
                    self.events.send(Event::Advanced(advanced));
                }
            }
            Command::StopAdvancing => {
                if self.pacer.is_advancing() {
                    self.publish()?;
                    self.end_advance();
                }
            }
            Command::ReportTicks => self.report_ticks(),
            Command::Sync => {
                let changes = self.simulation.sync_all()?;
                self.events.send(Event::Synced(changes));
            }
            Command::Stop { restore } => {
                self.end_advance();
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
    fn now(&self) -> Duration {
        self.origin.elapsed()
    }

    fn next_deadline(&self) -> Option<Instant> {
        if self.pacer.is_advancing() || self.publish_now {
            return Some(Instant::now());
        }
        let send = self.sends.next_send().filter(|_| self.unsent_ticks > 0);
        [self.pacer.next_tick(), send]
            .into_iter()
            .flatten()
            .min()
            .map(|due| self.origin + due)
    }

    fn run_ticks(&mut self) -> Result<(), EngineError> {
        let Some(due) = self.pacer.due(self.now()) else {
            return Ok(());
        };
        let ran = self.run_batch(due.limit, due.until_settled)?;
        if let Some(advanced) = self.pacer.ran(ran.ticks, ran.settled, self.now()) {
            self.publish()?;
            self.events.send(Event::Advanced(advanced));
        }
        Ok(())
    }

    fn end_advance(&mut self) {
        if let Some(advanced) = self.pacer.advance(None, self.now()) {
            self.events.send(Event::Advanced(advanced));
        }
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
        let due = self.unsent_ticks > 0 && self.sends.is_due(self.now());
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
        self.sends.sent(self.now());
        Ok(())
    }

    fn report_ticks(&mut self) {
        let now = Instant::now();
        self.events.send(Event::Ticked(TickReport {
            ticks: self.tally.ticks,
            ticking: self.tally.ticking,
            period: now - self.tally.since,
            advanced: self.pacer.advancing(),
            behind: self.pacer.is_behind(),
        }));
        self.tally = Tally::new(now);
    }
}
