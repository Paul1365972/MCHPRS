use super::{Session, SessionContext, SessionError, Stopped, Update};
use crate::circuit::NodeId;
use crate::engine::{Drive, Engine, EngineError, Snapshot};
use mchprs_redstone::pace::{Goal, Pacer, TickRate};
use std::any::Any;
use std::io;
use std::mem;
use std::ops::ControlFlow;
use std::panic::{self, AssertUnwindSafe};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::thread;
use std::time::{Duration, Instant};

const BATCH_TIME: Duration = Duration::from_millis(10);

const COMMAND_BATCH: usize = 64;

const MAX_CHUNK: u64 = 1024;

type Frame = Result<Update, SessionError>;

enum Command {
    SetTickRate(TickRate),
    Advance(Option<Goal>),
    Drive(Drive),
    Poll,
    Flush(Sender<()>),
    Stop {
        reset: bool,
        reply: Sender<Snapshot>,
    },
}

pub struct Worker {
    commands: Sender<Command>,
    frames: Receiver<Frame>,
}

impl Worker {
    pub fn spawn<E: Engine + 'static>(engine: E, context: SessionContext) -> io::Result<Worker> {
        let (commands, command_receiver) = mpsc::channel();
        let (frame_sender, frames) = mpsc::channel();
        thread::Builder::new()
            .name(format!("{} redpiler", context.name))
            .spawn(move || run(engine, &command_receiver, &frame_sender))?;
        Ok(Worker { commands, frames })
    }

    fn send(&self, command: Command) {
        let _ = self.commands.send(command);
    }

    fn receive(&self) -> Result<Option<Update>, SessionError> {
        let mut received: Option<Update> = None;
        for frame in self.frames.try_iter() {
            let frame = frame?;
            match &mut received {
                Some(update) => update.append(frame),
                None => received = Some(frame),
            }
        }
        Ok(received)
    }

    fn ended(&self) -> SessionError {
        self.frames.iter().find_map(Result::err).unwrap_or_else(|| {
            SessionError::Panicked("the worker thread ended without a reason".to_owned())
        })
    }
}

impl Session for Worker {
    fn set_tick_rate(&mut self, rate: TickRate) {
        self.send(Command::SetTickRate(rate));
    }

    fn advance(&mut self, goal: Option<Goal>) {
        self.send(Command::Advance(goal));
    }

    fn drive(&mut self, drive: Drive) {
        self.send(Command::Drive(drive));
    }

    fn poll(&mut self) -> Result<Option<Update>, SessionError> {
        let update = self.receive()?;
        match self.commands.send(Command::Poll) {
            Ok(()) => Ok(update),
            Err(_) => Err(self.ended()),
        }
    }

    fn flush(&mut self, timeout: Duration) -> Result<Update, SessionError> {
        let (reply, flushed) = mpsc::channel();
        self.send(Command::Flush(reply));
        match flushed.recv_timeout(timeout) {
            Ok(()) => Ok(self.receive()?.unwrap_or_default()),
            Err(RecvTimeoutError::Timeout) => Err(SessionError::Unresponsive),
            Err(RecvTimeoutError::Disconnected) => Err(self.ended()),
        }
    }

    fn stop(self: Box<Self>, reset: bool, timeout: Duration) -> Result<Stopped, SessionError> {
        let (reply, stopped) = mpsc::channel();
        self.send(Command::Stop { reset, reply });
        match stopped.recv_timeout(timeout) {
            Ok(snapshot) => Ok(Stopped {
                update: self.receive()?.unwrap_or_default(),
                snapshot,
            }),
            Err(RecvTimeoutError::Timeout) => Err(SessionError::Unresponsive),
            Err(RecvTimeoutError::Disconnected) => Err(self.ended()),
        }
    }
}

fn run<E: Engine>(engine: E, commands: &Receiver<Command>, frames: &Sender<Frame>) {
    let mut runner = Runner {
        engine,
        frames,
        origin: Instant::now(),
        pacer: Pacer::new(TickRate::Paused, Duration::ZERO),
        pending: Update::default(),
        wanted: false,
    };
    let ran = panic::catch_unwind(AssertUnwindSafe(|| runner.run(commands)));
    let error = match ran {
        Ok(Ok(())) => return,
        Ok(Err(error)) => SessionError::Failed(error),
        Err(payload) => SessionError::Panicked(panic_message(&*payload)),
    };
    let _ = frames.send(Err(error));
}

fn panic_message(payload: &(dyn Any + Send)) -> String {
    match payload.downcast_ref::<&str>() {
        Some(message) => (*message).to_owned(),
        None => payload
            .downcast_ref::<String>()
            .cloned()
            .unwrap_or_else(|| "no message".to_owned()),
    }
}

struct Runner<'a, E> {
    engine: E,
    frames: &'a Sender<Frame>,
    origin: Instant,
    pacer: Pacer,
    pending: Update,
    wanted: bool,
}

impl<E: Engine> Runner<'_, E> {
    fn run(&mut self, commands: &Receiver<Command>) -> Result<(), EngineError> {
        loop {
            let first = match self.next_tick() {
                Some(tick) => {
                    match commands.recv_timeout(tick.saturating_duration_since(Instant::now())) {
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
            let received = first.into_iter().chain(commands.try_iter());
            for command in received.take(COMMAND_BATCH) {
                if self.handle(command)?.is_break() {
                    return Ok(());
                }
            }
            self.run_ticks()?;
            self.cut()?;
        }
    }

    fn now(&self) -> Duration {
        self.origin.elapsed()
    }

    fn next_tick(&self) -> Option<Instant> {
        if self.pacer.is_advancing() {
            return Some(Instant::now());
        }
        self.pacer.next_tick().map(|due| self.origin + due)
    }

    fn handle(&mut self, command: Command) -> Result<ControlFlow<()>, EngineError> {
        match command {
            Command::SetTickRate(rate) => self.pacer.set_rate(rate, self.now()),
            Command::Advance(goal) => self.advance(goal),
            Command::Drive(drive) => {
                self.engine.drive(drive)?;
                self.pending.driven = true;
            }
            Command::Poll => self.wanted = true,
            Command::Flush(reply) => {
                let mut update = self.take_frame()?;
                let states = self.engine.snapshot()?.states.into_iter().enumerate();
                let states = states.map(|(index, state)| (NodeId::new(index), state));
                update.changes.states.extend(states);
                self.deliver(update);
                let _ = reply.send(());
            }
            Command::Stop { reset, reply } => {
                self.advance(None);
                let update = self.take_frame()?;
                self.deliver(update);
                if reset {
                    self.engine.reset()?;
                }
                let _ = reply.send(self.engine.snapshot()?);
                return Ok(ControlFlow::Break(()));
            }
        }
        Ok(ControlFlow::Continue(()))
    }

    fn advance(&mut self, goal: Option<Goal>) {
        if let Some(advanced) = self.pacer.advance(goal, self.now()) {
            self.pending.advanced.push(advanced);
        }
    }

    fn run_ticks(&mut self) -> Result<(), EngineError> {
        let Some(due) = self.pacer.due(self.now()) else {
            return Ok(());
        };
        let ran = self.run_batch(due.limit, due.until_settled)?;
        if let Some(advanced) = self.pacer.ran(ran.ticks, ran.settled, self.now()) {
            self.pending.advanced.push(advanced);
        }
        Ok(())
    }

    fn run_batch(&mut self, limit: u64, until_settled: bool) -> Result<Ran, EngineError> {
        let start = Instant::now();
        let mut ran = 0;
        let mut settled = false;
        let mut chunk = 1;
        while ran < limit {
            let ticks = chunk.min(limit - ran);
            if until_settled {
                if let Some(taken) = self.engine.settle(ticks)? {
                    ran += taken;
                    settled = true;
                    break;
                }
            } else {
                self.engine.run(ticks)?;
            }
            ran += ticks;
            let elapsed = start.elapsed();
            if elapsed >= BATCH_TIME {
                break;
            }
            let rate = ran as f64 / elapsed.as_secs_f64();
            let fitting = (rate * (BATCH_TIME - elapsed).as_secs_f64()) as u64;
            chunk = chunk.saturating_mul(2).min(fitting).clamp(1, MAX_CHUNK);
        }
        self.pending.ticks += ran;
        self.pending.ticking += start.elapsed();
        Ok(Ran {
            ticks: ran,
            settled,
        })
    }

    fn cut(&mut self) -> Result<(), EngineError> {
        let urgent = self.pending.driven || !self.pending.advanced.is_empty();
        if urgent || self.wanted && self.pending.ticks > 0 {
            let update = self.take_frame()?;
            self.deliver(update);
        }
        Ok(())
    }

    fn take_frame(&mut self) -> Result<Update, EngineError> {
        let mut update = mem::take(&mut self.pending);
        update.behind = self.pacer.is_behind();
        update.advancing = self.pacer.advancing();
        self.engine.changes(&mut update.changes)?;
        Ok(update)
    }

    fn deliver(&mut self, update: Update) {
        self.wanted = false;
        let _ = self.frames.send(Ok(update));
    }
}

struct Ran {
    ticks: u64,
    settled: bool,
}
