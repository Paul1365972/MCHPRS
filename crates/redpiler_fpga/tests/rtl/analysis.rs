use crate::scenario::{Step, CASES};
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::graph::{LinkKind, NodeKind};
use mchprs_redpiler::nodes::Snapshot;
use mchprs_redpiler_phase::engine::PhaseEngine;
use mchprs_simulation::Simulation;

fn watch_phase(circuit: &Circuit, steps: &[Step], mut watch: impl FnMut(&Snapshot)) {
    let mut phase = PhaseEngine::new(circuit);
    watch(&phase.complete_snapshot());
    for &step in steps {
        match step {
            Step::Run(ticks) => {
                for _ in 0..ticks {
                    phase.run_tick_watched(|phase| watch(&phase.complete_snapshot()));
                }
            }
            Step::Drive(node, input) => {
                phase.drive(node, input).expect("phase takes the drive");
                watch(&phase.complete_snapshot());
            }
        }
    }
}

fn is_pending(snapshot: &Snapshot, node: NodeId) -> bool {
    let mut pending_ticks = snapshot.pending_ticks.iter();
    pending_ticks.any(|&(pending, _)| pending == node)
}

fn default_input_is_on(circuit: &Circuit, snapshot: &Snapshot, node: NodeId) -> bool {
    let links = circuit.inputs(node).iter();
    let mut links = links.filter(|link| link.kind == LinkKind::Default);
    links.any(|link| snapshot.states[link.source.index()].power.get() > link.weight)
}

#[test]
fn the_analysis_holds_while_phase_runs_the_circuits() {
    for case in CASES {
        let circuit = case.circuit();
        watch_phase(&circuit, &case.steps(&circuit), |snapshot| {
            assert_states_are_not_stale(&circuit, snapshot);
        });
    }
}

fn assert_states_are_not_stale(circuit: &Circuit, snapshot: &Snapshot) {
    for node in circuit.ids() {
        let names = &circuit.node(node).names;
        let power = snapshot.states[node.index()].power;
        match circuit.node(node).kind {
            NodeKind::Button { .. } | NodeKind::PressurePlate => {
                assert!(
                    power.is_zero() || is_pending(snapshot, node),
                    "{names:?} stays pressed"
                );
            }
            NodeKind::Lamp => assert!(
                !power.is_zero() || !default_input_is_on(circuit, snapshot, node),
                "{names:?} is dark"
            ),
            _ => {}
        }
    }
}
