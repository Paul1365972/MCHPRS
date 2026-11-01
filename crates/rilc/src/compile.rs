use crate::input::Input;
use crate::numbers::{change, counted};
use crate::setup::Setup;
use crate::{Error, Outcome};
use mchprs_redpiler::compile::{compile_graph, optimize, BuildContext, CompileError, StageReport};
use mchprs_redpiler::extract::ExtractOptions;
use mchprs_redpiler::graph::Graph;
use mchprs_redpiler::passes::Pass;
use mchprs_ril::Printed;
use rustc_hash::FxHashMap;
use std::fs;
use std::io::{self, Write as _};
use std::path::Path;
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy)]
pub struct TimedStage {
    pub report: StageReport,
    pub time: Duration,
}

struct StageTimer<'a> {
    stages: Vec<TimedStage>,
    mark: Instant,
    inspect: &'a mut dyn FnMut(&StageReport, &Graph),
}

impl<'a> StageTimer<'a> {
    fn new(inspect: &'a mut dyn FnMut(&StageReport, &Graph)) -> StageTimer<'a> {
        StageTimer {
            stages: Vec::new(),
            mark: Instant::now(),
            inspect,
        }
    }

    fn finished(&mut self, report: &StageReport, graph: &Graph) {
        let time = self.mark.elapsed();
        self.stages.push(TimedStage {
            report: *report,
            time,
        });
        (self.inspect)(report, graph);
        self.mark = Instant::now();
    }
}

pub fn compile(
    input: &Input,
    setup: &Setup,
    inspect: &mut dyn FnMut(&StageReport, &Graph),
) -> Result<(Graph, Vec<TimedStage>), Error> {
    let plan = setup.plan()?;
    if let Input::Circuit(_) = input {
        setup.check_for_circuit()?;
    }
    compile_input(input, &plan.extract, &plan.passes, inspect)
}

pub fn compile_input(
    input: &Input,
    extract: &ExtractOptions,
    passes: &[&dyn Pass],
    inspect: &mut dyn FnMut(&StageReport, &Graph),
) -> Result<(Graph, Vec<TimedStage>), Error> {
    let mut timer = StageTimer::new(inspect);
    let graph = match input {
        Input::World(world) => {
            let finished = &mut |report: &StageReport, graph: &Graph| timer.finished(report, graph);
            let context = &mut BuildContext::silent().inspecting(finished);
            let region = world.blocks().region();
            let ticks = world.pending_ticks();
            compile_graph(world, region, ticks, extract, passes, context)?.0
        }
        Input::Circuit(circuit) => {
            let mut graph = circuit.graph(extract);
            timer.finished(&StageReport::new("parsed", &graph), &graph);
            let finished = &mut |report: &StageReport, graph: &Graph| timer.finished(report, graph);
            let context = &mut BuildContext::silent().inspecting(finished);
            optimize(&mut graph, passes, context).map_err(CompileError::from)?;
            graph
        }
    };
    Ok((graph, timer.stages))
}

pub fn command(
    path: &Path,
    setup: &Setup,
    circuit: Option<&str>,
    print_after_all: bool,
    output: Option<&Path>,
) -> Outcome {
    let input = Input::read(path, circuit)?;
    let mut stage_circuits = String::new();
    let mut names = StageNames::default();
    let (graph, stages) = compile(&input, setup, &mut |stage, graph| {
        if print_after_all {
            let name = names.next(stage.name);
            stage_circuits.push_str(&format!("\n{}\n", Printed::new(graph, &name)));
        }
    })?;

    let mut ril = format!("# setup: {setup}\n# input: {}", path.display());
    let name = match &input {
        Input::World(_) => circuit_name(path),
        Input::Circuit(circuit) => {
            ril.push_str(&format!(", circuit @{}", circuit.name));
            circuit.name.clone()
        }
    };
    ril.push('\n');
    ril.push_str(&stage_lines(&stages));
    let queued = graph
        .node_indices()
        .filter(|&node| graph[node].pending_tick.is_some())
        .count() as u64;
    match queued {
        0 => {}
        1 => ril.push_str("# 1 queued tick is left out, as RIL can't hold it\n"),
        _ => ril.push_str(&format!(
            "# {} are left out, as RIL can't hold them\n",
            counted(queued, "queued tick")
        )),
    }
    if print_after_all {
        ril.push_str(&stage_circuits);
    } else {
        ril.push_str(&format!("\n{}\n", Printed::new(&graph, &name)));
    }

    match output {
        Some(output) => fs::write(output, ril)
            .map_err(|error| format!("couldn't write {}: {error}", output.display()))?,
        None => io::stdout().write_all(ril.as_bytes())?,
    }
    Ok(true)
}

pub fn stage_lines(stages: &[TimedStage]) -> String {
    let mut lines = String::new();
    let mut previous: Option<&StageReport> = None;
    for TimedStage { report, time } in stages {
        lines.push_str(&format!(
            "# {}: {}, {}",
            report.name,
            counted(report.nodes as u64, "node"),
            counted(report.links as u64, "link")
        ));
        if let Some(previous) = previous {
            let mut changes = Vec::new();
            let node_change = report.nodes as i64 - previous.nodes as i64;
            let link_change = report.links as i64 - previous.links as i64;
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
        lines.push_str(&format!(" in {time:.1?}\n"));
        previous = Some(report);
    }
    lines
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
