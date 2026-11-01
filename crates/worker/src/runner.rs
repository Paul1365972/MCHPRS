use super::*;
use crate::pacing::Pacing;

const BATCH_TIME: Duration = Duration::from_millis(10);

pub(super) fn run<F>(
    build: F,
    tick_rate: TickRate,
    send_rate: SendRate,
    command_receiver: Receiver<Command>,
    event_sender: Sender<Event>,
) where
    F: FnOnce(&mut dyn FnMut(Stage) -> ControlFlow<()>) -> Result<Simulation, EngineError>,
{
    let mut events = Events(event_sender);
    let mut pacing = Pacing::new(tick_rate, send_rate);
    let Some((simulation, deferred)) =
        build_simulation(build, &command_receiver, &mut events, &mut pacing)
    else {
        return;
    };
    events.send(Event::Started(simulation.circuit().clone()));
    pacing.forget_backlog();
    let mut runner = Runner {
        simulation,
        events,
        pacing,
        advances: VecDeque::new(),
        ticks_since_batch: 0,
        publish_now: false,
    };
    if let Err(error) = runner.run(deferred, &command_receiver) {
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
    request: RequestId,
    remaining: u64,
    inputs_after: Vec<(BlockPos, Input)>,
}

struct Runner {
    simulation: Simulation,
    events: Events,
    pacing: Pacing,
    advances: VecDeque<Advance>,
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
                || !self.advances.is_empty()
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
            Command::Drive { position, input } => match self.advances.back_mut() {
                Some(advance) => advance.inputs_after.push((position, input)),
                None => self.drive(position, input)?,
            },
            Command::Advance { request, ticks } => self.advances.push_back(Advance {
                request,
                remaining: ticks,
                inputs_after: Vec::new(),
            }),
            Command::Sync => {
                let changes = self.simulation.sync_all()?;
                let ticks = self.simulation.pending_ticks()?;
                self.events.send(Event::Synced { changes, ticks });
            }
            Command::Stop { restore } => {
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
        if let Some(remaining) = self.advances.front().map(|advance| advance.remaining) {
            let ran = self.run_batch(remaining)?;
            if ran < remaining {
                self.advances[0].remaining = remaining - ran;
                return Ok(());
            }
            let advance = self.advances.pop_front().expect("an advance is running");
            self.publish()?;
            self.events.send(Event::Advanced(advance.request));
            for (position, input) in advance.inputs_after {
                self.drive(position, input)?;
            }
            self.pacing.next_tick = Instant::now();
            return Ok(());
        }
        let ticks = self.pacing.due_ticks(Instant::now());
        if ticks > 0 {
            let ran = self.run_batch(ticks)?;
            self.pacing.ran_ticks(ran);
        }
        Ok(())
    }

    /// Runs up to `limit` ticks and returns how many ran. It stops after `BATCH_TIME`, in chunks
    /// sized to the time left at the speed so far but at most doubling from one tick, so a tick
    /// that suddenly costs more can't stall commands.
    fn run_batch(&mut self, limit: u64) -> Result<u64, EngineError> {
        let start = Instant::now();
        let mut ran = 0;
        let mut chunk = 1;
        while ran < limit {
            let ticks = chunk.min(limit - ran);
            self.simulation.run(ticks)?;
            ran += ticks;
            let elapsed = start.elapsed();
            if elapsed >= BATCH_TIME {
                break;
            }
            let rate = ran as f64 / elapsed.as_secs_f64();
            let fitting = (rate * (BATCH_TIME - elapsed).as_secs_f64()) as u64;
            chunk = chunk.saturating_mul(2).min(fitting).max(1);
        }
        self.ticks_since_batch += ran;
        Ok(ran)
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
        let ticks = std::mem::take(&mut self.ticks_since_batch);
        if ticks > 0 || !changes.is_empty() {
            self.events.send(Event::Batch { changes, ticks });
        }
        if let Some(interval) = self.pacing.send_interval {
            self.pacing.next_send = Instant::now() + interval;
        }
        Ok(())
    }
}
