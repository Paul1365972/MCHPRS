//! Gives a stepped simulation a thread of its own.

use crate::handle::{self, Request, Updates};
use crate::input::DriveError;
use crate::schedule::{Chunk, Schedule};
use crate::ticks::SimulationError;
use crate::{Command, Event, Running, Simulation, Space, Update, View};
use std::mem;
use std::sync::mpsc::{Receiver, RecvTimeoutError, TryRecvError};
use std::time::Instant;

/// Fails where no thread can be spawned.
pub fn start<S: Space>(
    simulation: Box<dyn Simulation<S>>,
    name: &str,
) -> Result<Box<dyn Running<S>>, SimulationError> {
    let capabilities = simulation.capabilities();
    handle::spawn(name, capabilities, move |requests, updates| {
        let stepper = Stepper {
            simulation,
            schedule: Schedule::default(),
            pending: Update::default(),
            landed: false,
            updates,
        };
        stepper.serve(&requests)
    })
}

struct Stepper<S: Space> {
    simulation: Box<dyn Simulation<S>>,
    schedule: Schedule,
    pending: Update<S>,
    landed: bool,
    updates: Updates<S>,
}

impl<S: Space> Stepper<S> {
    fn serve(mut self, requests: &Receiver<Request<S>>) -> Result<(), SimulationError> {
        let mut now = Instant::now();
        loop {
            let request = match self.schedule.next_tick() {
                None => requests.recv().ok(),
                Some(tick) if tick > now => match requests.recv_timeout(tick - now) {
                    Ok(request) => Some(request),
                    Err(RecvTimeoutError::Timeout) => {
                        now = Instant::now();
                        None
                    }
                    Err(RecvTimeoutError::Disconnected) => return Ok(()),
                },
                Some(_) => match requests.try_recv() {
                    Ok(request) => Some(request),
                    Err(TryRecvError::Empty) => None,
                    Err(TryRecvError::Disconnected) => return Ok(()),
                },
            };
            match request {
                Some(Request::Command(command)) => {
                    now = Instant::now();
                    self.handle(command, now)?;
                }
                Some(Request::Stop(reply)) => {
                    self.deliver();
                    let _ = reply.send(self.simulation.stop());
                    return Ok(());
                }
                None if self.schedule.next_tick().is_none() => return Ok(()),
                None => {
                    let simulation = &mut self.simulation;
                    let (_, end) =
                        self.schedule
                            .run(now, &mut self.pending.events, |chunk| match chunk {
                                Chunk::Run(ticks) => simulation.run(ticks),
                                Chunk::Settle(limit) => simulation.settle(limit),
                            })?;
                    now = end;
                }
            }
            self.deliver();
        }
    }

    fn handle(
        &mut self,
        command: Command<S::Address>,
        now: Instant,
    ) -> Result<(), SimulationError> {
        let events = &mut self.pending.events;
        match command {
            Command::Pause => self.schedule.pause(events),
            Command::Run(run) => self.schedule.set(run, now, events),
            Command::Drive(at, input) => match self.simulation.drive(at, input) {
                Ok(()) => {
                    self.simulation
                        .show(View::Observed, &mut self.pending.changes)?;
                    self.landed = true;
                }
                Err(DriveError::Refused(error)) => events.push(Event::Refused(at, error)),
                Err(DriveError::Simulation(error)) => return Err(error),
            },
            Command::Watch(watch) => self.simulation.watch(watch)?,
            Command::Show(view) => {
                self.simulation.show(view, &mut self.pending.changes)?;
                events.push(Event::Shown(view));
            }
        }
        Ok(())
    }

    fn deliver(&mut self) {
        if self.pending.events.is_empty() && !self.landed {
            return;
        }
        let mut update = mem::take(&mut self.pending);
        update.ticks = self.simulation.ticks();
        update.behind = self.schedule.is_behind();
        self.landed = false;
        let _ = self.updates.send(Ok(update));
    }
}
