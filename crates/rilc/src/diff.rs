use crate::experiment::{write_report, Compiled, Loaded, Result, Schedule};
use mchprs_redpiler::compile_graph::NodeState;
use mchprs_redpiler::{Engine, NodeId, Reporting};
use mchprs_world::TickPriority;
use serde_json::{json, Value};
use std::collections::{BTreeMap, HashMap};
use std::io::Write;
use std::path::Path;

const REPORTED_NODES: usize = 50;

struct Mirror {
    backend: String,
    states: Vec<NodeState>,
    pending: HashMap<NodeId, Vec<(u32, TickPriority)>>,
}

impl Mirror {
    fn refresh(&mut self, engine: &mut dyn Engine) -> Result<()> {
        let (changes, pending_ticks) = engine.snapshot()?;
        for (node, state) in changes.states {
            self.states[node.index()] = state;
        }
        self.pending.clear();
        for tick in pending_ticks {
            self.pending
                .entry(tick.node)
                .or_default()
                .push((tick.delay, tick.priority));
        }
        for ticks in self.pending.values_mut() {
            ticks.sort_unstable();
        }
        Ok(())
    }

    fn describe(&self, node: NodeId) -> Value {
        let state = self.states[node.index()];
        json!({
            "powered": state.is_powered(),
            "locked": state.repeater_locked,
            "strength": state.power.get(),
            "pending": self.pending.get(&node).map(Vec::as_slice).unwrap_or_default(),
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
    let mut compiled = loaded.compile();
    let initial: Vec<NodeState> = compiled
        .netlist
        .ids()
        .map(|id| NodeState {
            pending_tick: false,
            ..compiled.netlist.node(id).state
        })
        .collect();
    let mut engines = backends
        .iter()
        .map(|backend| compiled.engine(backend, Reporting::All))
        .collect::<Result<Vec<_>>>()?;
    let mut mirrors: Vec<Mirror> = backends
        .iter()
        .map(|backend| Mirror {
            backend: backend.clone(),
            states: initial.clone(),
            pending: HashMap::new(),
        })
        .collect();

    let experiment = &loaded.experiment;
    let mut schedule = Schedule::new(experiment, Some(every));
    let mut identical_through = 0;
    let mut divergence = None;
    while let Some(tick) = schedule.advance(&compiled, &mut engines)? {
        for (mirror, engine) in mirrors.iter_mut().zip(&mut engines) {
            mirror.refresh(engine.as_mut())?;
        }
        let differing = differing_nodes(&mirrors, ignore_pending);
        if !differing.is_empty() {
            divergence = Some(report_divergence(&compiled, &mirrors, &differing, tick));
            break;
        }
        identical_through = tick;
        eprintln!("tick {tick}/{}: identical", experiment.ticks);
    }

    let identical = divergence.is_none();
    let details = json!({
        "backends": backends,
        "every": every,
        "ignore_pending": ignore_pending,
        "identical_through": identical_through,
        "divergence": divergence,
    });
    write_report(output, &loaded, &compiled, details)?;
    Ok(identical)
}

fn differing_nodes(mirrors: &[Mirror], ignore_pending: bool) -> Vec<NodeId> {
    let (first, rest) = mirrors.split_first().unwrap();
    (0..first.states.len())
        .map(NodeId::new)
        .filter(|node| {
            rest.iter().any(|other| {
                other.states[node.index()] != first.states[node.index()]
                    || (!ignore_pending && other.pending.get(node) != first.pending.get(node))
            })
        })
        .collect()
}

fn report_divergence(
    compiled: &Compiled,
    mirrors: &[Mirror],
    differing: &[NodeId],
    tick: u64,
) -> Value {
    let nodes: Vec<Value> = differing
        .iter()
        .take(REPORTED_NODES)
        .map(|&node| {
            let compiled_node = compiled.netlist.node(node);
            let states: BTreeMap<&str, Value> = mirrors
                .iter()
                .map(|mirror| (mirror.backend.as_str(), mirror.describe(node)))
                .collect();
            json!({
                "node": node.index(),
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
