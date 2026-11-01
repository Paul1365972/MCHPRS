use crate::backend::{BackendFailure, Batch, Snapshot};
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
    Snapshot(Sender<Snapshot>),
    Stop(Sender<Snapshot>),
}

struct Published {
    batch: Batch,
    tick_rate_version: u64,
}

pub struct Worker {
    requests: Sender<Request>,
    batches: Receiver<Published>,
    thread: JoinHandle<()>,
    tick_rate: TickRate,
    tick_rate_version: u64,
}

impl Worker {
    pub fn start(block_map: BlockMap, engine: Box<dyn Engine>) -> Worker {
        let (requests, worker_requests) = mpsc::channel();
        let (worker_batches, batches) = mpsc::channel();
        let name = match thread::current().name() {
            Some(plot) => format!("{plot} redpiler"),
            None => "redpiler".to_owned(),
        };
        let thread = thread::Builder::new()
            .name(name)
            .spawn(move || {
                WorkerThread::new(engine, block_map, worker_requests, worker_batches).run()
            })
            .expect("failed to spawn the redpiler thread");
        Worker {
            requests,
            batches,
            thread,
            tick_rate: TickRate::Paused,
            tick_rate_version: 0,
        }
    }

    pub fn set_tick_rate(&mut self, rate: TickRate) -> Result<(), BackendFailure> {
        if !rate.replaces(self.tick_rate) {
            return Ok(());
        }
        self.tick_rate = rate;
        self.tick_rate_version += 1;
        self.send(Request::SetTickRate(rate))
    }

    pub fn set_send_rate(&mut self, rate: SendRate) -> Result<(), BackendFailure> {
        self.send(Request::SetSendRate(rate))
    }

    pub fn input(&mut self, pos: BlockPos, input: Input) -> Result<(), BackendFailure> {
        self.send(Request::Input(pos, input))
    }

    pub fn next_batch(&mut self, timeout: Duration) -> Result<Option<Batch>, BackendFailure> {
        match self.batches.recv_timeout(timeout) {
            Ok(published) => Ok(Some(self.settle(published))),
            Err(RecvTimeoutError::Timeout) => Ok(None),
            Err(RecvTimeoutError::Disconnected) => Err(BackendFailure),
        }
    }

    pub fn snapshot(&mut self) -> Result<Snapshot, BackendFailure> {
        self.request_snapshot(Request::Snapshot)
    }

    pub fn stop(self) -> Result<Snapshot, BackendFailure> {
        let snapshot = self.request_snapshot(Request::Stop)?;
        self.thread.join().map_err(|_| BackendFailure)?;
        Ok(snapshot)
    }

    fn send(&self, request: Request) -> Result<(), BackendFailure> {
        self.requests.send(request).map_err(|_| BackendFailure)
    }

    fn request_snapshot(
        &self,
        request: fn(Sender<Snapshot>) -> Request,
    ) -> Result<Snapshot, BackendFailure> {
        let (reply, answer) = mpsc::channel();
        self.send(request(reply))?;
        let snapshot = answer.recv().map_err(|_| BackendFailure)?;
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
}

enum Exit {
    Stopped,
    Abandoned,
    Failed(EngineError),
}

fn proceed<T>(result: Result<T, EngineError>) -> ControlFlow<Exit, T> {
    match result {
        Ok(value) => ControlFlow::Continue(value),
        Err(error) => ControlFlow::Break(Exit::Failed(error)),
    }
}

struct WorkerThread {
    engine: Box<dyn Engine>,
    block_map: BlockMap,
    tick_schedule: TickSchedule,
    send_schedule: SendSchedule,
    tick_rate_version: u64,
    ticks_completed: u64,
    owed_ticks_finished: bool,
    publish_pending: bool,
    requests: Receiver<Request>,
    batches: Sender<Published>,
}

impl WorkerThread {
    fn new(
        engine: Box<dyn Engine>,
        block_map: BlockMap,
        requests: Receiver<Request>,
        batches: Sender<Published>,
    ) -> WorkerThread {
        let now = Instant::now();
        WorkerThread {
            engine,
            block_map,
            tick_schedule: TickSchedule::new(now),
            send_schedule: SendSchedule::new(SendRate::Never, now),
            tick_rate_version: 0,
            ticks_completed: 0,
            owed_ticks_finished: false,
            publish_pending: false,
            requests,
            batches,
        }
    }

    fn run(mut self) {
        let exit = loop {
            if let ControlFlow::Break(exit) = self.serve_slice() {
                break exit;
            }
        };
        match exit {
            Exit::Stopped | Exit::Abandoned => {}
            Exit::Failed(error) => error!("the redpiler engine failed: {error}"),
        }
    }

    fn serve_slice(&mut self) -> ControlFlow<Exit> {
        while let Some(request) = self.try_receive()? {
            self.handle(request)?;
        }
        let now = Instant::now();
        let request = match self.tick_schedule.next(now) {
            Step::Run(due) => {
                self.run_slice(due, now)?;
                None
            }
            Step::WaitFor(wait) => self.wait_for_request(Some(wait), now)?,
            Step::Idle => self.wait_for_request(None, now)?,
        };
        if let Some(request) = request {
            self.handle(request)?;
        }
        self.publish_if_due(Instant::now())
    }

    fn try_receive(&self) -> ControlFlow<Exit, Option<Request>> {
        match self.requests.try_recv() {
            Ok(request) => ControlFlow::Continue(Some(request)),
            Err(TryRecvError::Empty) => ControlFlow::Continue(None),
            Err(TryRecvError::Disconnected) => ControlFlow::Break(Exit::Abandoned),
        }
    }

    fn wait_for_request(
        &self,
        until_tick: Option<Duration>,
        now: Instant,
    ) -> ControlFlow<Exit, Option<Request>> {
        let until_send = self
            .publish_pending
            .then(|| self.send_schedule.until_send(now))
            .flatten();
        let received = match [until_tick, until_send].into_iter().flatten().min() {
            Some(wait) => self.requests.recv_timeout(wait),
            None => self.requests.recv().map_err(RecvTimeoutError::from),
        };
        match received {
            Ok(request) => ControlFlow::Continue(Some(request)),
            Err(RecvTimeoutError::Timeout) => ControlFlow::Continue(None),
            Err(RecvTimeoutError::Disconnected) => ControlFlow::Break(Exit::Abandoned),
        }
    }

    fn run_slice(&mut self, due: u64, now: Instant) -> ControlFlow<Exit> {
        let completed = proceed(self.engine.run_ticks(due, now + RUN_SLICE))?;
        let owed = self.tick_schedule.ticks_owed();
        self.tick_schedule.complete(completed);
        self.ticks_completed += completed;
        self.owed_ticks_finished |= owed != 0 && self.tick_schedule.ticks_owed() == 0;
        self.publish_pending |= completed != 0;
        ControlFlow::Continue(())
    }

    fn handle(&mut self, request: Request) -> ControlFlow<Exit> {
        match request {
            Request::SetTickRate(rate) => {
                self.tick_schedule.set_rate(rate, Instant::now());
                self.tick_rate_version += 1;
                self.publish_pending = true;
            }
            Request::SetSendRate(rate) => self.send_schedule.set_rate(rate),
            Request::Input(pos, input) => match self.block_map.node_accepting(pos, input) {
                Some(node) => {
                    proceed(self.engine.input(node, input))?;
                    self.publish_pending = true;
                }
                None => warn!("no node accepting {input:?} at {pos}"),
            },
            Request::Snapshot(reply) => self.reply_snapshot(reply)?,
            Request::Stop(reply) => {
                self.reply_snapshot(reply)?;
                return ControlFlow::Break(Exit::Stopped);
            }
        }
        ControlFlow::Continue(())
    }

    fn reply_snapshot(&mut self, reply: Sender<Snapshot>) -> ControlFlow<Exit> {
        let snapshot = proceed(self.snapshot())?;
        match reply.send(snapshot) {
            Ok(()) => ControlFlow::Continue(()),
            Err(_) => ControlFlow::Break(Exit::Abandoned),
        }
    }

    fn publish_if_due(&mut self, now: Instant) -> ControlFlow<Exit> {
        let due = self.owed_ticks_finished || self.send_schedule.is_due(now);
        if !self.publish_pending || !due {
            return ControlFlow::Continue(());
        }
        let changes = proceed(self.engine.take_changes())?;
        let batch = self.take_batch(&changes);
        self.send_schedule.record_send(now);
        let published = Published {
            batch,
            tick_rate_version: self.tick_rate_version,
        };
        match self.batches.send(published) {
            Ok(()) => ControlFlow::Continue(()),
            Err(_) => ControlFlow::Break(Exit::Abandoned),
        }
    }

    fn snapshot(&mut self) -> Result<Snapshot, EngineError> {
        let snapshot = self.engine.snapshot()?;
        Ok(Snapshot {
            changes: self.take_batch(&snapshot.changes),
            pending_ticks: self.block_map.tick_entries(&snapshot.pending_ticks),
        })
    }

    fn take_batch(&mut self, changes: &NodeChanges) -> Batch {
        let mut batch = self.block_map.translate(changes);
        batch.ticks_completed = std::mem::take(&mut self.ticks_completed);
        batch.ticks_owed = self.tick_schedule.ticks_owed();
        self.owed_ticks_finished = false;
        self.publish_pending = false;
        batch
    }
}
