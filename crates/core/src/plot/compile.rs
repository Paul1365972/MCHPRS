use super::world::PlotBlocks;
use anyhow::{anyhow, Result};
use mchprs_backends::Setup;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_redstone::tick_queue::TickEntry;
use mchprs_simulation::backend::{CompileContext, CompileStep, Source};
use mchprs_simulation::layout::Layout;
use mchprs_simulation::{Blocks, Running};
use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::sync::Arc;
use std::thread;

pub struct CompileJob {
    steps: Receiver<CompileStep>,
    step: Option<CompileStep>,
    compiled: Receiver<Result<Compilation>>,
    cancelled: Arc<AtomicBool>,
}

/// A simulation that runs by itself, started on the compile thread.
pub struct Compilation {
    pub simulation: Box<dyn Running<Blocks>>,
    pub layout: Layout,
}

impl CompileJob {
    pub fn spawn(
        world: Arc<PlotBlocks>,
        region: Region,
        ticks: Vec<TickEntry>,
        observed: Vec<BlockPos>,
        setup: Setup,
        name: String,
    ) -> io::Result<CompileJob> {
        let (step_sender, steps) = mpsc::channel();
        let (compiled_sender, compiled) = mpsc::channel();
        let cancelled = Arc::new(AtomicBool::new(false));
        let flag = cancelled.clone();
        thread::Builder::new()
            .name(format!("{name} compile"))
            .spawn(move || {
                let mut progress = |step| {
                    let _ = step_sender.send(step);
                };
                let mut context = CompileContext::new(&mut progress, &flag);
                let source = Source {
                    ticks: &ticks,
                    observed: &observed,
                    ..Source::new(&*world, region)
                };
                let compiled = setup.compile(&source, &mut context).map_err(Into::into);
                let compilation = compiled.and_then(|(simulation, layout)| {
                    let simulation = simulation.start()?;
                    Ok(Compilation { simulation, layout })
                });
                let _ = compiled_sender.send(compilation);
            })?;
        Ok(CompileJob {
            steps,
            step: None,
            compiled,
            cancelled,
        })
    }

    pub fn step(&self) -> Option<CompileStep> {
        self.step
    }

    pub fn progressed(&mut self) -> bool {
        let latest = self.steps.try_iter().last();
        self.step = latest.or(self.step);
        latest.is_some()
    }

    pub fn finished(&mut self) -> Option<Result<Compilation>> {
        match self.compiled.try_recv() {
            Ok(compiled) => Some(compiled),
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
