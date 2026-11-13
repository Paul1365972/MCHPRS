//! The owner's end of a simulation that runs on a thread of its own.

use crate::ticks::SimulationError;
use crate::{Capabilities, Command, End, Running, Space, Update};
use anyhow::anyhow;
use std::sync::mpsc::{self, Receiver, Sender, TryRecvError};
use std::thread;

pub type Updates<S> = Sender<Result<Update<S>, SimulationError>>;

pub enum Request<S: Space> {
    Command(Command<S::Address>),
    /// Answered once the simulation has ended.
    Stop(Sender<Result<End<S>, SimulationError>>),
}

/// Runs `serve` on a thread named `name`. An error it returns reaches the owner after every update
/// it sent.
pub fn spawn<S: Space>(
    name: &str,
    capabilities: Capabilities,
    serve: impl FnOnce(Receiver<Request<S>>, Updates<S>) -> Result<(), SimulationError> + Send + 'static,
) -> Result<Box<dyn Running<S>>, SimulationError> {
    let (requests, request_receiver) = mpsc::channel();
    let (update_sender, updates) = mpsc::channel();
    thread::Builder::new()
        .name(name.to_owned())
        .spawn(move || {
            let failure = update_sender.clone();
            if let Err(error) = serve(request_receiver, update_sender) {
                let _ = failure.send(Err(error));
            }
        })?;
    Ok(Box::new(Handle {
        capabilities,
        requests,
        updates,
    }))
}

struct Handle<S: Space> {
    capabilities: Capabilities,
    requests: Sender<Request<S>>,
    updates: Receiver<Result<Update<S>, SimulationError>>,
}

impl<S: Space> Handle<S> {
    fn ended(&self) -> SimulationError {
        let error = self.updates.try_iter().find_map(Result::err);
        error.unwrap_or_else(|| anyhow!("the thread of the simulation ended"))
    }
}

impl<S: Space> Running<S> for Handle<S> {
    fn capabilities(&self) -> Capabilities {
        self.capabilities
    }

    fn send(&mut self, command: Command<S::Address>) {
        let _ = self.requests.send(Request::Command(command));
    }

    fn poll(&mut self) -> Result<Option<Update<S>>, SimulationError> {
        let mut merged: Option<Update<S>> = None;
        loop {
            let update = match self.updates.try_recv() {
                Ok(update) => update?,
                Err(TryRecvError::Empty) => return Ok(merged),
                Err(TryRecvError::Disconnected) => return Err(self.ended()),
            };
            match &mut merged {
                Some(merged) => merged.append(update),
                None => merged = Some(update),
            }
        }
    }

    fn wait(&mut self) -> Result<Update<S>, SimulationError> {
        let mut update = match self.updates.recv() {
            Ok(update) => update?,
            Err(_) => return Err(self.ended()),
        };
        if let Some(later) = self.poll()? {
            update.append(later);
        }
        Ok(update)
    }

    fn stop(self: Box<Self>) -> Result<End<S>, SimulationError> {
        let (reply, ended) = mpsc::channel();
        let _ = self.requests.send(Request::Stop(reply));
        let Ok(end) = ended.recv() else {
            return Err(self.ended());
        };
        let mut end = end?;
        let mut unshown = S::Changes::default();
        for update in self.updates.try_iter() {
            unshown.extend([update?.changes]);
        }
        unshown.extend([end.changes]);
        end.changes = unshown;
        Ok(end)
    }
}
