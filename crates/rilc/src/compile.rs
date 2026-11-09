use crate::input::{self, Loaded};
use crate::numbers::{change, counted};
use crate::Error;
use mchprs_redpiler::backend::GraphContext;
use mchprs_redpiler::extract;
use mchprs_redpiler::graph::Graph;
use mchprs_ril::Options;
use rustc_hash::FxHashMap;
use std::time::{Duration, Instant};

type Inspect<'a> = &'a mut dyn FnMut(&'static str, &Graph);

#[derive(Debug, Clone, Copy)]
pub struct TimedStage {
    pub name: &'static str,
    pub nodes: usize,
    pub links: usize,
    pub time: Duration,
}

struct StageTimer<'a> {
    stages: Vec<TimedStage>,
    mark: Instant,
    inspect: Inspect<'a>,
}

impl<'a> StageTimer<'a> {
    fn new(inspect: Inspect<'a>) -> StageTimer<'a> {
        StageTimer {
            stages: Vec::new(),
            mark: Instant::now(),
            inspect,
        }
    }

    fn finished(&mut self, name: &'static str, graph: &Graph) {
        self.stages.push(TimedStage {
            name,
            nodes: graph.node_count(),
            links: graph.edge_count(),
            time: self.mark.elapsed(),
        });
        (self.inspect)(name, graph);
        self.mark = Instant::now();
    }
}

#[derive(Default)]
pub struct StageNames(FxHashMap<&'static str, usize>);

impl StageNames {
    pub fn next(&mut self, stage: &'static str) -> String {
        let runs = self.0.entry(stage).or_default();
        *runs += 1;
        let name = stage.replace('-', "_");
        match *runs {
            1 => name,
            runs => format!("{name}_{runs}"),
        }
    }
}

pub fn compile_test(
    input: &Loaded,
    options: &Options,
    inspect: Inspect<'_>,
) -> Result<(Graph, Vec<TimedStage>), Error> {
    timed(inspect, |context| {
        let mut graph = match input {
            Loaded::World(world) => {
                let graph = extract::extract(&input::source(world), &options.extract)?;
                context.inspect("extracted", &graph);
                graph
            }
            Loaded::Circuit(circuit) => {
                let mut graph = circuit.graph();
                options.extract.observe(&mut graph);
                context.inspect("parsed", &graph);
                graph
            }
        };
        context.run_passes(&mut graph, &options.passes)?;
        Ok(graph)
    })
}

fn timed(
    inspect: Inspect<'_>,
    compile: impl FnOnce(&mut GraphContext<'_>) -> Result<Graph, Error>,
) -> Result<(Graph, Vec<TimedStage>), Error> {
    let mut timer = StageTimer::new(inspect);
    let finished = &mut |name, graph: &Graph| timer.finished(name, graph);
    let graph = compile(&mut GraphContext::silent().inspecting(finished))?;
    Ok((graph, timer.stages))
}

pub fn stage_lines(stages: &[TimedStage]) -> String {
    let mut lines = String::new();
    let mut previous: Option<&TimedStage> = None;
    for stage in stages {
        lines.push_str(&format!(
            "# {}: {}, {}",
            stage.name,
            counted(stage.nodes as u64, "node"),
            counted(stage.links as u64, "link")
        ));
        if let Some(previous) = previous {
            let mut changes = Vec::new();
            let node_change = stage.nodes as i64 - previous.nodes as i64;
            let link_change = stage.links as i64 - previous.links as i64;
            if node_change != 0 {
                changes.push(change(node_change, "node"));
            }
            if link_change != 0 {
                changes.push(change(link_change, "link"));
            }
            if changes.is_empty() {
                changes.push("unchanged".to_owned());
            }
            lines.push_str(&format!(" ({})", changes.join(", ")));
        }
        lines.push_str(&format!(" in {:.1?}\n", stage.time));
        previous = Some(stage);
    }
    lines
}
