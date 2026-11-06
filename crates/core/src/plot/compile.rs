use anyhow::anyhow;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::region::Region;
use mchprs_redpiler::backend::CompilePlan;
use mchprs_redpiler::block_map::BlockMap;
use mchprs_redpiler::compile::{compile, BuildContext, Stage};
use mchprs_redpiler::session::{Session, SessionContext};
use mchprs_redpiler_backends::BackendSetup;
use mchprs_redstone::tick_queue::TickEntry;
use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::sync::Arc;
use std::thread;

pub struct Started {
    pub blocks: Box<BlockMap>,
    pub session: Box<dyn Session>,
}

pub struct CompileJob {
    stages: Receiver<Stage>,
    stage: Option<Stage>,
    started: Receiver<anyhow::Result<Started>>,
    cancelled: Arc<AtomicBool>,
}

impl CompileJob {
    pub fn spawn(
        world: impl BlockView + Send + 'static,
        region: Region,
        ticks: Vec<TickEntry>,
        setup: BackendSetup,
        plan: CompilePlan,
        session: SessionContext,
    ) -> io::Result<CompileJob> {
        let (stage_sender, stages) = mpsc::channel();
        let (start_sender, started) = mpsc::channel();
        let cancelled = Arc::new(AtomicBool::new(false));
        let flag = cancelled.clone();
        thread::Builder::new()
            .name(format!("{} compile", session.name))
            .spawn(move || {
                let mut progress = |stage| {
                    let _ = stage_sender.send(stage);
                };
                let mut context = BuildContext::new(&mut progress, &flag);
                let started = start(&world, region, &ticks, &setup, plan, &mut context, session);
                let _ = start_sender.send(started);
            })?;
        Ok(CompileJob {
            stages,
            stage: None,
            started,
            cancelled,
        })
    }

    pub fn stage(&self) -> Option<Stage> {
        self.stage
    }

    pub fn progressed(&mut self) -> bool {
        let latest = self.stages.try_iter().last();
        self.stage = latest.or(self.stage);
        latest.is_some()
    }

    pub fn finished(&mut self) -> Option<anyhow::Result<Started>> {
        match self.started.try_recv() {
            Ok(started) => Some(started),
            Err(TryRecvError::Empty) => None,
            Err(TryRecvError::Disconnected) => {
                Some(Err(anyhow!("the compile thread stopped unexpectedly")))
            }
        }
    }
}

impl Drop for CompileJob {
    fn drop(&mut self) {
        self.cancelled.store(true, Ordering::Relaxed);
    }
}

fn start(
    world: &impl BlockView,
    region: Region,
    ticks: &[TickEntry],
    setup: &BackendSetup,
    plan: CompilePlan,
    context: &mut BuildContext<'_>,
    session: SessionContext,
) -> anyhow::Result<Started> {
    let circuit = compile(world, region, ticks, &plan.extract, &plan.passes, context)?.circuit;
    let session = setup
        .backend()
        .start(&circuit, setup.flags(), context, session)?;
    Ok(Started {
        blocks: Box::new(BlockMap::new(circuit)),
        session,
    })
}
