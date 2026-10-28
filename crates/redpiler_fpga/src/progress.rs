use mchprs_redpiler::backend::GraphContext;
use mchprs_simulation::backend::{Cancelled, CompileStep};
use std::time::Instant;
use tracing::info;

pub struct Progress<'a, 'b> {
    context: &'a mut GraphContext<'b>,
    stages: usize,
    started: usize,
    running: Option<(&'static str, Instant)>,
}

impl<'a, 'b> Progress<'a, 'b> {
    pub fn new(context: &'a mut GraphContext<'b>, stages: usize) -> Progress<'a, 'b> {
        Progress {
            context,
            stages,
            started: 0,
            running: None,
        }
    }

    pub fn start(&mut self, stage: &'static str) -> Result<(), Cancelled> {
        self.finish();
        self.running = Some((stage, Instant::now()));
        let completed = self.started;
        self.started += 1;
        self.context.report(CompileStep {
            name: stage,
            completed,
            total: self.stages,
        })
    }

    pub fn finish(&mut self) {
        if let Some((stage, started)) = self.running.take() {
            info!("{stage} took {:.1?}", started.elapsed());
        }
    }

    pub fn is_cancelled(&self) -> bool {
        self.context.is_cancelled()
    }
}
