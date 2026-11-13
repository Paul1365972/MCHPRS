use super::reference::ReferenceEngine;
use super::trace::{NodeView, PendingTicks, Seen, View};
use super::{play, Oracle, Step, Verdict};
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::graph::NodeState;
use mchprs_redstone::input::Input;
use mchprs_ril::Module;

fn circuit(ril: &str) -> Circuit {
    let module = Module::parse(ril).expect("the circuit parses");
    Circuit::new(module.circuits()[0].graph()).expect("the circuit is valid")
}

fn node(circuit: &Circuit, name: &str) -> NodeId {
    let mut nodes = circuit.ids();
    let node = nodes.find(|&node| circuit.node(node).names == [name]);
    node.expect("the circuit has the node")
}

/// The comparator outputs 5 - 0. Updated after either input from the lever changes, it sees 15 - 0
/// or 5 - 10 and schedules a tick; updated once after both, it sees 15 - 10 and schedules none.
const DOUBLE_FED_COMPARATOR: &str = "
circuit @double_fed_comparator {
  %constant = constant(power=5) -> [%comparator:0]
  %lever = lever -> [%comparator:0, side %comparator:5]
  %comparator = comparator(mode=subtract, power=5)
}
";

#[test]
fn updates_a_target_once_after_both_its_inputs_change() {
    let circuit = circuit(DOUBLE_FED_COMPARATOR);
    let all_nodes: Vec<NodeId> = circuit.ids().collect();
    let lever = node(&circuit, "lever");
    let view = |states: &[NodeState]| View {
        nodes: states
            .iter()
            .map(|&state| NodeView {
                state,
                played_note: false,
            })
            .collect(),
        pending_ticks: Some(PendingTicks::new(Vec::new())),
    };
    let mut states = circuit.initial().states;
    let before = view(&states);
    states[lever.index()] = NodeState::from_powered(true);
    let trace = [
        Seen {
            steps_played: 0,
            view: before,
        },
        Seen {
            steps_played: 1,
            view: view(&states),
        },
    ];
    let steps = [Step::Drive(lever, Input::Toggle)];
    let verdict = Oracle::new(&circuit, &all_nodes).check(&steps, &trace);
    assert!(matches!(verdict, Ok(Verdict::Passed)));
}

const TWO_TORCHES: &str = "
circuit @two_torches {
  %both = lever -> [%first:0, %second:0]
  %early = lever -> [%first:0]
  %late = lever -> [%second:0]
  %first = torch(powered)
  %second = torch(powered)
}
";

#[test]
fn runs_ticks_in_the_order_they_were_scheduled_unless_one_change_scheduled_them() {
    let circuit = circuit(TWO_TORCHES);
    let toggle = |lever: &str| Step::Drive(node(&circuit, lever), Input::Toggle);
    let may_run_reversed = |steps: &[Step]| {
        let mut engine = ReferenceEngine::new(&circuit);
        for &step in steps {
            let played = play(&mut engine, step);
            played.expect("the reference engine never fails");
        }
        let mut pending_ticks = engine.complete_snapshot().pending_ticks;
        assert_eq!(pending_ticks.len(), 2);
        pending_ticks.reverse();
        let view = |pending_ticks| View {
            nodes: Vec::new(),
            pending_ticks: Some(PendingTicks::new(pending_ticks)),
        };
        let trace = [
            Seen {
                steps_played: 0,
                view: view(Vec::new()),
            },
            Seen {
                steps_played: steps.len(),
                view: view(pending_ticks),
            },
        ];
        Oracle::new(&circuit, &[]).check(steps, &trace).is_ok()
    };
    assert!(!may_run_reversed(&[toggle("early"), toggle("late")]));
    assert!(may_run_reversed(&[toggle("both")]));
}
