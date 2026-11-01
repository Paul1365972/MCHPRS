use crate::backend::{Backend, BackendFailure, Batch, Snapshot};
use crate::block_map::BlockMap;
use crate::engine::{Engine, EngineError, Input, NodeChanges};
use mchprs_blocks::BlockPos;
use mchprs_world::{SendRate, SendSchedule, Step, TickRate, TickSchedule};
use std::ops::ControlFlow;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender, TryRecvError};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};
use tracing::{error, warn};

const RUN_SLICE: Duration = Duration::from_millis(5);

enum Request {
    SetTickRate(TickRate),
    SetSendRate(SendRate),
    Input(BlockPos, Input),
    Snapshot,
    Stop,
}

struct Published {
    batch: Batch,
    tick_rate_version: u64,
}

pub struct Worker {
    requests: Sender<Request>,
    batches: Receiver<Published>,
    snapshots: Receiver<Snapshot>,
    thread: Option<JoinHandle<()>>,
    tick_rate: TickRate,
    tick_rate_version: u64,
}

impl Worker {
    pub fn start(block_map: BlockMap, engine: Box<dyn Engine>) -> Worker {
        let (requests, worker_requests) = mpsc::channel();
        let (worker_batches, batches) = mpsc::channel();
        let (worker_snapshots, snapshots) = mpsc::channel();
        let name = match thread::current().name() {
            Some(plot) => format!("{plot} redpiler"),
            None => "redpiler".to_owned(),
        };
        let thread = thread::Builder::new()
            .name(name)
            .spawn(move || {
                WorkerThread::new(
                    engine,
                    block_map,
                    worker_requests,
                    worker_batches,
                    worker_snapshots,
                )
                .run()
            })
            .expect("failed to spawn the redpiler thread");
        Worker {
            requests,
            batches,
            snapshots,
            thread: Some(thread),
            tick_rate: TickRate::Paused,
            tick_rate_version: 0,
        }
    }

    fn send(&mut self, request: Request) -> Result<(), BackendFailure> {
        if self.thread.is_none() {
            return Err(BackendFailure);
        }
        if self.requests.send(request).is_err() {
            return Err(self.fail());
        }
        Ok(())
    }

    fn request_snapshot(&mut self, request: Request) -> Result<Snapshot, BackendFailure> {
        self.send(request)?;
        let Ok(snapshot) = self.snapshots.recv() else {
            return Err(self.fail());
        };
        let mut changes = Batch::default();
        while let Ok(published) = self.batches.try_recv() {
            changes.append(self.settle(published));
        }
        changes.append(snapshot.changes);
        Ok(Snapshot {
            changes,
            pending_ticks: snapshot.pending_ticks,
        })
    }

    fn settle(&self, published: Published) -> Batch {
        let mut batch = published.batch;
        if published.tick_rate_version < self.tick_rate_version {
            batch.ticks_owed = self.tick_rate.ticks_owed();
        }
        batch
    }

    fn fail(&mut self) -> BackendFailure {
        if let Some(thread) = self.thread.take() {
            match thread.join() {
                Ok(()) => error!("the redpiler thread stopped on its own"),
                Err(_) => error!("the redpiler thread panicked"),
            }
        }
        BackendFailure
    }
}

impl Backend for Worker {
    fn set_tick_rate(&mut self, rate: TickRate) {
        if !rate.replaces(self.tick_rate) {
            return;
        }
        self.tick_rate = rate;
        self.tick_rate_version += 1;
        let _ = self.send(Request::SetTickRate(rate));
    }

    fn set_send_rate(&mut self, rate: SendRate) {
        let _ = self.send(Request::SetSendRate(rate));
    }

    fn input(&mut self, pos: BlockPos, input: Input) {
        let _ = self.send(Request::Input(pos, input));
    }

    fn next_batch(&mut self, timeout: Duration) -> Result<Option<Batch>, BackendFailure> {
        if self.thread.is_none() {
            return Err(BackendFailure);
        }
        match self.batches.recv_timeout(timeout) {
            Ok(published) => Ok(Some(self.settle(published))),
            Err(RecvTimeoutError::Timeout) => Ok(None),
            Err(RecvTimeoutError::Disconnected) => Err(self.fail()),
        }
    }

    fn snapshot(&mut self) -> Result<Snapshot, BackendFailure> {
        self.request_snapshot(Request::Snapshot)
    }

    fn stop(mut self: Box<Self>) -> Result<Snapshot, BackendFailure> {
        let snapshot = self.request_snapshot(Request::Stop)?;
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
        Ok(snapshot)
    }
}

enum Exit {
    Stopped,
    Abandoned,
    Failed(EngineError),
}

struct WorkerThread {
    engine: Box<dyn Engine>,
    block_map: BlockMap,
    tick_schedule: TickSchedule,
    send_schedule: SendSchedule,
    tick_rate_version: u64,
    ticks_completed: u64,
    publish_pending: bool,
    requests: Receiver<Request>,
    batches: Sender<Published>,
    snapshots: Sender<Snapshot>,
}

impl WorkerThread {
    fn new(
        engine: Box<dyn Engine>,
        block_map: BlockMap,
        requests: Receiver<Request>,
        batches: Sender<Published>,
        snapshots: Sender<Snapshot>,
    ) -> WorkerThread {
        let now = Instant::now();
        WorkerThread {
            engine,
            block_map,
            tick_schedule: TickSchedule::new(now),
            send_schedule: SendSchedule::new(SendRate::Never, now),
            tick_rate_version: 0,
            ticks_completed: 0,
            publish_pending: false,
            requests,
            batches,
            snapshots,
        }
    }

    fn run(mut self) {
        let failure = match self.serve() {
            Exit::Stopped => match self.snapshot() {
                Ok(snapshot) => {
                    let _ = self.snapshots.send(snapshot);
                    return;
                }
                Err(error) => error,
            },
            Exit::Abandoned => return,
            Exit::Failed(error) => error,
        };
        error!("the redpiler engine failed: {failure}");
    }

    fn serve(&mut self) -> Exit {
        loop {
            loop {
                match self.requests.try_recv() {
                    Ok(request) => match self.handle(request) {
                        Ok(ControlFlow::Continue(())) => {}
                        Ok(ControlFlow::Break(())) => return Exit::Stopped,
                        Err(error) => return Exit::Failed(error),
                    },
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => return Exit::Abandoned,
                }
            }

            let now = Instant::now();
            let received = match self.tick_schedule.next(now) {
                Step::Run(due) => {
                    if let Err(error) = self.run_slice(due, now) {
                        return Exit::Failed(error);
                    }
                    Err(RecvTimeoutError::Timeout)
                }
                Step::WaitFor(wait) => self.wait_for_request(Some(wait), now),
                Step::Idle => self.wait_for_request(None, now),
            };
            match received {
                Ok(request) => match self.handle(request) {
                    Ok(ControlFlow::Continue(())) => {}
                    Ok(ControlFlow::Break(())) => return Exit::Stopped,
                    Err(error) => return Exit::Failed(error),
                },
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => return Exit::Abandoned,
            }

            if let Err(error) = self.publish_if_due(Instant::now()) {
                return Exit::Failed(error);
            }
        }
    }

    fn run_slice(&mut self, due: u64, now: Instant) -> Result<(), EngineError> {
        let completed = self.engine.run_ticks(due, now + RUN_SLICE)?;
        self.tick_schedule.complete(completed);
        self.ticks_completed += completed;
        self.publish_pending |= completed != 0;
        Ok(())
    }

    fn wait_for_request(
        &mut self,
        until_tick: Option<Duration>,
        now: Instant,
    ) -> Result<Request, RecvTimeoutError> {
        let until_send = self
            .publish_pending
            .then(|| self.send_schedule.until_send(now))
            .flatten();
        match [until_tick, until_send].into_iter().flatten().min() {
            Some(wait) => self.requests.recv_timeout(wait),
            None => self
                .requests
                .recv()
                .map_err(|_| RecvTimeoutError::Disconnected),
        }
    }

    fn handle(&mut self, request: Request) -> Result<ControlFlow<()>, EngineError> {
        match request {
            Request::SetTickRate(rate) => {
                self.tick_schedule.set_rate(rate, Instant::now());
                self.tick_rate_version += 1;
                self.publish_pending = true;
            }
            Request::SetSendRate(rate) => self.send_schedule.set_rate(rate),
            Request::Input(pos, input) => match self.block_map.node_accepting(pos, input) {
                Some(node) => {
                    self.engine.input(node, input)?;
                    self.publish_pending = true;
                }
                None => warn!("no node accepting {input:?} at {pos}"),
            },
            Request::Snapshot => {
                let snapshot = self.snapshot()?;
                let _ = self.snapshots.send(snapshot);
            }
            Request::Stop => return Ok(ControlFlow::Break(())),
        }
        Ok(ControlFlow::Continue(()))
    }

    fn publish_if_due(&mut self, now: Instant) -> Result<(), EngineError> {
        if !self.publish_pending || !self.send_schedule.is_due(now) {
            return Ok(());
        }
        let changes = self.engine.take_changes()?;
        let batch = self.batch(&changes);
        self.send_schedule.record_send(now);
        let _ = self.batches.send(Published {
            batch,
            tick_rate_version: self.tick_rate_version,
        });
        Ok(())
    }

    fn snapshot(&mut self) -> Result<Snapshot, EngineError> {
        let (changes, pending_ticks) = self.engine.snapshot()?;
        Ok(Snapshot {
            changes: self.batch(&changes),
            pending_ticks: self.block_map.translate_ticks(&pending_ticks),
        })
    }

    fn batch(&mut self, changes: &NodeChanges) -> Batch {
        let mut batch = self.block_map.translate(changes);
        batch.ticks_completed = std::mem::take(&mut self.ticks_completed);
        batch.ticks_owed = self.tick_schedule.ticks_owed();
        self.publish_pending = false;
        batch
    }
}
