mod pacing;
mod runner;

use mchprs_blocks::BlockPos;
use mchprs_redpiler::block_map::BlockChanges;
use mchprs_redpiler::circuit::Circuit;
use mchprs_redpiler::compile::Stage;
use mchprs_redpiler::engine::{DriveError, EngineError, Input};
use mchprs_redpiler::simulation::Simulation;
use mchprs_redstone::tick_queue::TickEntry;
use std::collections::VecDeque;
use std::ops::ControlFlow;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender, TryRecvError};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TickRate {
    Paused,
    PerSecond(f64),
    Unlimited,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SendRate {
    pub per_second: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RequestId(u64);

/// The ticks run since the previous report, and how long running them took.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct TickReport {
    pub ticks: u64,
    pub ticking: Duration,
}

pub enum Event {
    Progress(Stage),
    Started(Arc<Circuit>),
    Ticked(TickReport),
    Batch(BlockChanges),
    Advanced(RequestId),
    Rejected {
        position: BlockPos,
        error: DriveError,
    },
    Synced {
        changes: BlockChanges,
        ticks: Vec<TickEntry>,
    },
    Stopped {
        changes: BlockChanges,
        ticks: Vec<TickEntry>,
    },
    Cancelled,
    Failed(EngineError),
}

impl Event {
    pub fn is_last(&self) -> bool {
        matches!(
            self,
            Event::Stopped { .. } | Event::Cancelled | Event::Failed(_)
        )
    }
}

enum Command {
    SetTickRate(TickRate),
    SetSendRate(SendRate),
    Drive {
        position: BlockPos,
        input: Input,
    },
    Advance {
        request: RequestId,
        ticks: u64,
    },
    ReportTicks,
    Sync,
    /// Advances that haven't finished are dropped, with the inputs waiting for them.
    Stop {
        restore: bool,
    },
}

/// Dropping the worker stops it without waiting for it.
pub struct Worker {
    commands: Sender<Command>,
    events: Receiver<Event>,
    next_request: u64,
    thread: Option<JoinHandle<()>>,
    finished: bool,
}

impl Worker {
    pub fn spawn<F>(tick_rate: TickRate, send_rate: SendRate, build: F) -> Worker
    where
        F: FnOnce(&mut dyn FnMut(Stage) -> ControlFlow<()>) -> Result<Simulation, EngineError>
            + Send
            + 'static,
    {
        let (commands, command_receiver) = mpsc::channel();
        let (event_sender, events) = mpsc::channel();
        let thread = thread::Builder::new()
            .name("redpiler worker".to_owned())
            .spawn(move || runner::run(build, tick_rate, send_rate, command_receiver, event_sender))
            .expect("the worker thread starts");
        Worker {
            commands,
            events,
            next_request: 0,
            thread: Some(thread),
            finished: false,
        }
    }

    /// The events sent since the last call. A worker that went away without its last event ends
    /// with [`Event::Failed`].
    pub fn poll(&mut self) -> Vec<Event> {
        let mut events = Vec::new();
        loop {
            match self.events.try_recv() {
                Ok(event) => self.record(&mut events, event),
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    self.fail_unless_finished(&mut events);
                    break;
                }
            }
        }
        events
    }

    /// Asks for an [`Event::Ticked`] about the ticks run since the previous one.
    pub fn report_ticks(&self) {
        self.send(Command::ReportTicks);
    }

    pub fn set_tick_rate(&self, rate: TickRate) {
        self.send(Command::SetTickRate(rate));
    }

    pub fn set_send_rate(&self, rate: SendRate) {
        self.send(Command::SetSendRate(rate));
    }

    pub fn drive(&self, position: BlockPos, input: Input) {
        self.send(Command::Drive { position, input });
    }

    /// Runs `ticks` more ticks as fast as possible, then sends [`Event::Advanced`]. Inputs sent
    /// afterwards wait for it.
    pub fn advance(&mut self, ticks: u64) -> RequestId {
        let request = RequestId(self.next_request);
        self.next_request += 1;
        self.send(Command::Advance { request, ticks });
        request
    }

    pub fn sync(&mut self) -> Vec<Event> {
        self.send(Command::Sync);
        self.collect_until(|event| matches!(event, Event::Synced { .. }))
    }

    pub fn stop(&mut self, restore: bool) -> Vec<Event> {
        self.send(Command::Stop { restore });
        let events = self.collect_until(|_| false);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
        events
    }

    fn send(&self, command: Command) {
        // The worker only goes away after its last event or by panicking, which ends the events
        let _ = self.commands.send(command);
    }

    fn collect_until(&mut self, done: impl Fn(&Event) -> bool) -> Vec<Event> {
        let mut collected = Vec::new();
        loop {
            let Ok(event) = self.events.recv() else {
                self.fail_unless_finished(&mut collected);
                break;
            };
            let finished = done(&event) || event.is_last();
            self.record(&mut collected, event);
            if finished {
                break;
            }
        }
        collected
    }

    fn record(&mut self, events: &mut Vec<Event>, event: Event) {
        self.finished |= event.is_last();
        events.push(event);
    }

    fn fail_unless_finished(&mut self, events: &mut Vec<Event>) {
        if !self.finished {
            self.finished = true;
            events.push(Event::Failed(
                "the worker thread stopped unexpectedly".into(),
            ));
        }
    }
}
