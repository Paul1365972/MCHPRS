mod common;

use common::{graph, node_named, Run};
use mchprs_redpiler::circuit::NodeId;
use mchprs_redpiler::graph::{Graph, NodeState};
use mchprs_redpiler::nodes::Changes;
use mchprs_redpiler::passes::Pass;
use mchprs_redpiler_phase::coalesce::Coalesce;
use mchprs_redstone::input::Input;
use mchprs_redstone::tick_queue::PendingTick;
use mchprs_simulation::{Simulation, View};
use std::collections::BTreeMap;

const ROUNDS: usize = 3;
const TICKS_AFTER_DRIVE: usize = 7;

#[derive(Debug, Clone, Default, PartialEq, Eq)]
struct NodeReport {
    state: NodeState,
    pending_ticks: Vec<PendingTick>,
    reported_state: Option<NodeState>,
    played_note: bool,
}

impl Run {
    fn drive(&mut self, name: &str) {
        let node = self.node(name);
        let input_kinds = self.circuit.input_kinds();
        let mut inputs = Input::all().filter(|&input| input_kinds.accept(node, input).is_ok());
        let input = inputs.next().expect("the node takes an input");
        self.engine.drive(node, input).expect("the engine drives");
    }

    fn shown(&mut self) -> BTreeMap<String, NodeReport> {
        let snapshot = self.engine.complete_snapshot();
        let mut changes = Changes::default();
        let reported = self.engine.show(View::Observed, &mut changes);
        reported.expect("the engine reports its changes");
        let of_state = |&state| NodeReport {
            state,
            ..NodeReport::default()
        };
        let mut shown: Vec<NodeReport> = snapshot.states.iter().map(of_state).collect();
        for (node, tick) in snapshot.pending_ticks {
            shown[node.index()].pending_ticks.push(tick);
        }
        for (node, state) in changes.states {
            shown[node.index()].reported_state = Some(state);
        }
        for node in changes.played_note_blocks {
            shown[node.index()].played_note = true;
        }
        let shown = &shown;
        let named = |node: NodeId| {
            let names = self.circuit.node(node).names.iter();
            names.map(move |name| (name.clone(), shown[node.index()].clone()))
        };
        self.circuit.ids().flat_map(named).collect()
    }
}

fn merged(graph: Graph) -> Vec<String> {
    let inputs = graph.node_indices().filter(|&node| graph[node].is_input());
    let inputs: Vec<String> = inputs.map(|node| graph[node].names[0].clone()).collect();
    let mut coalesced = graph.clone();
    Coalesce.run(&mut coalesced);
    let nodes = coalesced.node_indices().map(|node| &coalesced[node].names);
    let merged = nodes.filter(|names| names.len() > 1);
    let merged = merged.map(|names| names.join(" ")).collect();

    let mut runs = [Run::new(graph), Run::new(coalesced)];
    assert_show_the_same(&mut runs);
    for _ in 0..ROUNDS {
        for input in inputs.iter().map(Some).chain([None]) {
            if let Some(input) = input {
                runs.iter_mut().for_each(|run| run.drive(input));
                assert_show_the_same(&mut runs);
            }
            for _ in 0..TICKS_AFTER_DRIVE {
                runs.iter_mut().for_each(Run::tick);
                assert_show_the_same(&mut runs);
            }
        }
    }
    merged
}

fn assert_show_the_same(runs: &mut [Run; 2]) {
    let [unmerged, merged] = runs.each_mut().map(Run::shown);
    assert_eq!(unmerged, merged);
}

#[test]
fn twins_and_twins_of_twins_merge() {
    let merged = merged(graph(
        "circuit @twins_of_twins {
           %lever = lever -> [%torch1:0, %torch2:0]
           %torch1 = torch(powered) -> [%repeater1:0]
           %torch2 = torch(powered) -> [%repeater2:0]
           %repeater1 = repeater(delay=1, powered) -> [%lamp:0]
           %repeater2 = repeater(delay=1, powered) -> [%lamp:0]
           %lamp = lamp(powered)
         }",
    ));
    assert_eq!(merged, ["torch1 torch2", "repeater1 repeater2"]);
}

#[test]
fn twins_in_loops_merge() {
    let merged = merged(graph(
        "circuit @clocks {
           %torch1 = torch(powered) -> [%repeater1:0]
           %repeater1 = repeater(delay=1) tick(delay=1, priority=high, order=0) -> [%torch1:0]
           %torch2 = torch(powered) -> [%repeater2:0]
           %repeater2 = repeater(delay=1) tick(delay=1, priority=high, order=1) -> [%torch2:0]
         }",
    ));
    assert_eq!(merged, ["torch1 torch2", "repeater1 repeater2"]);
}

#[test]
fn twins_powering_one_input_keep_the_lower_weight() {
    let merged = merged(graph(
        "circuit @weights {
           %lever = lever -> [%far:0, %near:0]
           %far = comparator -> [%reader:3]
           %near = comparator -> [%reader:1]
           %reader = comparator
         }",
    ));
    assert_eq!(merged, ["far near"]);
}

#[test]
fn nodes_merge_only_with_the_same_pending_tick() {
    let merged = merged(graph(
        "circuit @pending_ticks {
           %lever = lever(powered) -> [%soon1:0, %soon2:0, %later:0]
           %soon1 = repeater(delay=2) tick(delay=1, priority=high, order=0)
           %soon2 = repeater(delay=2) tick(delay=1, priority=high, order=1)
           %later = repeater(delay=2) tick(delay=2, priority=high)
         }",
    ));
    assert_eq!(merged, ["soon1 soon2"]);
}

#[test]
fn nodes_merge_only_if_both_are_observed_or_neither() {
    let mut graph = graph(
        "circuit @observed {
           %lever = lever -> [%seen1:0, %seen2:0, %unseen:0]
           %seen1 = torch(powered)
           %seen2 = torch(powered)
           %unseen = torch(powered)
         }",
    );
    let unseen = node_named(&graph, "unseen");
    graph[unseen].observed = false;
    assert_eq!(merged(graph), ["seen1 seen2"]);
}
