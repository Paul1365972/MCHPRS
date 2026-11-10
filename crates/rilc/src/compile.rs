use crate::dot::Dot;
use crate::input::{self, Loaded};
use crate::numbers::{change, counted};
use crate::setup::Setup;
use crate::{Error, Outcome};
use anyhow::Context;
use mchprs_redpiler::backend::GraphContext;
use mchprs_redpiler::extract;
use mchprs_redpiler::graph::Graph;
use mchprs_ril::{Options, Printed};
use rustc_hash::FxHashMap;
use std::fs;
use std::io::{self, Write as _};
use std::path::Path;
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

pub fn command(
    path: &Path,
    setup: &Setup,
    circuit: Option<&str>,
    print_after_all: bool,
    dot: bool,
    output: Option<&Path>,
) -> Outcome {
    let input = Loaded::read(path, circuit)?;
    let write = |graph: &Graph, name: &str| {
        if dot {
            Dot::new(graph, name).to_string()
        } else {
            Printed::new(graph, name).to_string()
        }
    };
    let mut stage_circuits = String::new();
    let mut names = StageNames::default();
    let (graph, stages) = compile(&input, setup, &mut |stage, graph| {
        if print_after_all {
            let name = names.next(stage);
            stage_circuits.push_str(&format!("\n{}\n", write(graph, &name)));
        }
    })?;

    let mut ril = format!("# setup: {setup}\n# input: {}", path.display());
    let name = match &input {
        Loaded::World(_) => circuit_name(path),
        Loaded::Circuit(circuit) => {
            ril.push_str(&format!(", circuit @{}", circuit.name));
            circuit.name.clone()
        }
    };
    ril.push('\n');
    ril.push_str(&stage_lines(&stages));
    if print_after_all {
        ril.push_str(&stage_circuits);
    } else {
        ril.push_str(&format!("\n{}\n", write(&graph, &name)));
    }

    match output {
        Some(output) => fs::write(output, ril)
            .with_context(|| format!("couldn't write {}", output.display()))?,
        None => io::stdout().write_all(ril.as_bytes())?,
    }
    Ok(true)
}

pub fn compile(
    input: &Loaded,
    setup: &Setup,
    inspect: Inspect<'_>,
) -> Result<(Graph, Vec<TimedStage>), Error> {
    timed(inspect, |context| match input {
        Loaded::World(world) => setup.world_graph(world, context),
        Loaded::Circuit(circuit) => setup.circuit_graph(circuit.graph(), context),
    })
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

fn circuit_name(path: &Path) -> String {
    let stem = path.file_stem().map(|stem| stem.to_string_lossy());
    let name: String = stem
        .unwrap_or_default()
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
        .collect();
    if name.is_empty() {
        "circuit".to_owned()
    } else {
        name
    }
}
