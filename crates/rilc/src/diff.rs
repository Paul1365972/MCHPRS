use crate::experiment::{boundaries, run_until, Compiled, Loaded, Result};
use mchprs_redpiler::compile_graph::NodeState;
use mchprs_redpiler::{Engine, NodeId};
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::io::Write;
use std::path::Path;

const REPORTED_NODES: usize = 50;

struct Mirror {
    backend: String,
    engine: Box<dyn Engine>,
    states: Vec<NodeState>,
    pending: BTreeMap<usize, Vec<(u32, String)>>,
}

impl Mirror {
    fn refresh(&mut self) -> Result<()> {
        let (changes, pending_ticks) = self.engine.snapshot()?;
        for (node, state) in changes.states {
            self.states[node.index()] = state;
        }
        self.pending.clear();
        for tick in pending_ticks {
            let entry = self.pending.entry(tick.node.index()).or_default();
            entry.push((tick.delay, format!("{:?}", tick.priority)));
            entry.sort();
        }
        Ok(())
    }

    fn describe(&self, node: usize) -> Value {
        let state = self.states[node];
        json!({
            "powered": state.is_powered(),
            "locked": state.repeater_locked,
            "strength": state.power.get(),
            "pending": self.pending.get(&node).cloned().unwrap_or_default(),
        })
    }
}

pub fn diff(
    path: &Path,
    backends: &[String],
    every: u64,
    ignore_pending: bool,
    output: &mut dyn Write,
) -> Result<bool> {
    if backends.len() < 2 {
        return Err("diff needs at least two backends".into());
    }
    let loaded = Loaded::open(path)?;
    let compiled = loaded.compile();
    let initial: Vec<NodeState> = compiled
        .netlist
        .ids()
        .map(|id| NodeState {
            pending_tick: false,
            ..compiled.netlist.node(id).state
        })
        .collect();
    let mut mirrors = backends
        .iter()
        .map(|backend| {
            Ok(Mirror {
                backend: backend.clone(),
                engine: compiled.engine(backend, false)?,
                states: initial.clone(),
                pending: BTreeMap::new(),
            })
        })
        .collect::<Result<Vec<_>>>()?;

    let experiment = &loaded.experiment;
    let mut inputs = experiment.inputs.iter().peekable();
    let mut completed = 0;
    let mut identical_through = 0;
    let mut divergence = None;
    for boundary in boundaries(experiment, Some(every)) {
        for mirror in &mut mirrors {
            run_until(mirror.engine.as_mut(), boundary - completed)?;
        }
        completed = boundary;
        while let Some(input) = inputs.next_if(|input| input.tick == completed) {
            let node = compiled.node_at(*input)?;
            for mirror in &mut mirrors {
                mirror.engine.input(node, input.input())?;
            }
        }
        for mirror in &mut mirrors {
            mirror.refresh()?;
        }
        let differing = differing_nodes(&mirrors, ignore_pending);
        if !differing.is_empty() {
            divergence = Some(report_divergence(
                &compiled, &mirrors, &differing, completed,
            ));
            break;
        }
        identical_through = completed;
        eprintln!("tick {completed}/{}: identical", experiment.ticks);
    }

    let report = json!({
        "rilc": env!("GIT_HASH").trim(),
        "experiment": loaded.path,
        "plot_sha256": loaded.plot_sha256,
        "backends": backends,
        "options": experiment.options,
        "every": every,
        "ignore_pending": ignore_pending,
        "nodes": compiled.netlist.len(),
        "stages": compiled.stages.borrow().as_slice(),
        "identical_through": identical_through,
        "divergence": divergence,
    });
    serde_json::to_writer(&mut *output, &report)?;
    writeln!(output)?;
    Ok(report["divergence"].is_null())
}

fn differing_nodes(mirrors: &[Mirror], ignore_pending: bool) -> Vec<usize> {
    let (first, rest) = mirrors.split_first().unwrap();
    (0..first.states.len())
        .filter(|&node| {
            rest.iter().any(|other| {
                other.states[node] != first.states[node]
                    || (!ignore_pending && other.pending.get(&node) != first.pending.get(&node))
            })
        })
        .collect()
}

fn report_divergence(
    compiled: &Compiled,
    mirrors: &[Mirror],
    differing: &[usize],
    tick: u64,
) -> Value {
    let nodes: Vec<Value> = differing
        .iter()
        .take(REPORTED_NODES)
        .map(|&node| {
            let compiled_node = compiled.netlist.node(NodeId::new(node));
            let states: BTreeMap<&str, Value> = mirrors
                .iter()
                .map(|mirror| (mirror.backend.as_str(), mirror.describe(node)))
                .collect();
            json!({
                "node": node,
                "kind": format!("{:?}", compiled_node.ty),
                "positions": compiled_node.block.iter().map(|(position, _)| position).collect::<Vec<_>>(),
                "states": states,
            })
        })
        .collect();
    json!({
        "tick": tick,
        "differing_nodes": differing.len(),
        "nodes": nodes,
    })
}
