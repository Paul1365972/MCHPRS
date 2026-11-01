use crate::scenario::{build, complete_snapshot, Step, CASES};
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::graph::NodeState;
use mchprs_redpiler::nodes::{Changes, Nodes, Snapshot};
use mchprs_redpiler_fpga::backend::FpgaBackend;
use mchprs_redpiler_phase::backend::PhaseBackend;
use mchprs_redstone::tick_queue::PendingTick;
use mchprs_simulation::readback::Readback;
use mchprs_simulation::Simulation;
use std::collections::BTreeSet;
use std::fmt::Debug;

const SETTLE_LIMIT: u64 = 50;

pub type View = Vec<(NodeState, BTreeSet<PendingTick>, bool)>;

type ObservedView = Vec<(NodeId, NodeState, bool)>;

type Reported = [Vec<NodeState>; 2];

fn played_note_blocks(engine: &mut dyn Simulation<Nodes>) -> Vec<NodeId> {
    let mut changes = Changes::default();
    let reported = engine.show(mchprs_simulation::View::Observed, &mut changes);
    reported.expect("the engine reports its changes");
    changes.played_note_blocks
}

fn view_of(snapshot: Snapshot, engine: &mut dyn Simulation<Nodes>) -> View {
    let states = snapshot.states.into_iter();
    let mut view: View = states
        .map(|state| (state, BTreeSet::new(), false))
        .collect();
    for (node, tick) in snapshot.pending_ticks {
        view[node.index()].1.insert(tick);
    }
    for node in played_note_blocks(engine) {
        view[node.index()].2 = true;
    }
    view
}

pub fn view(engine: &mut dyn Simulation<Nodes>) -> View {
    view_of(complete_snapshot(engine), engine)
}

fn observed_view(
    circuit: &Circuit,
    engine: &mut dyn Simulation<Nodes>,
    reported: &mut [NodeState],
) -> ObservedView {
    let mut changes = Changes::default();
    let changed = engine.show(mchprs_simulation::View::Observed, &mut changes);
    changed.expect("the engine reports its changes");
    for (node, state) in changes.states {
        reported[node.index()] = state.shown();
    }
    let observed = circuit.ids().filter(|&node| circuit.node(node).observed);
    let mut view: ObservedView = observed
        .map(|node| (node, reported[node.index()], false))
        .collect();
    for node in changes.played_note_blocks {
        let observed = view.iter_mut().find(|(observed, _, _)| *observed == node);
        observed.expect("only an observed node reports a play").2 = true;
    }
    view
}

fn assert_states_are_as_reported(
    circuit: &Circuit,
    phase: &mut dyn Simulation<Nodes>,
    reported: &Reported,
) {
    let states = complete_snapshot(phase).states;
    for reported in reported {
        for node in circuit.ids().filter(|&node| circuit.node(node).observed) {
            let names = &circuit.node(node).names;
            assert_eq!(
                states[node.index()].shown(),
                reported[node.index()],
                "{names:?}"
            );
        }
    }
}

fn assert_same_view(
    circuit: &Circuit,
    engines: &mut [&mut dyn Simulation<Nodes>; 2],
    reported: &mut Reported,
    after: &dyn Debug,
) {
    let [phase, fpga] = engines;
    let [phase_reported, fpga_reported] = reported;
    match fpga.read().expect("the engine reads its state") {
        Readback::Complete(snapshot) => {
            let views = view(&mut **phase)
                .into_iter()
                .zip(view_of(snapshot, &mut **fpga));
            for (node, (phase, fpga)) in circuit.ids().zip(views) {
                let names = &circuit.node(node).names;
                assert_eq!(phase, fpga, "{names:?} after {after:?}");
            }
        }
        Readback::Incomplete => {
            let views = observed_view(circuit, &mut **phase, phase_reported).into_iter();
            for (phase, fpga) in views.zip(observed_view(circuit, &mut **fpga, fpga_reported)) {
                let names = &circuit.node(phase.0).names;
                assert_eq!(phase, fpga, "{names:?} after {after:?}");
            }
        }
    }
}

fn assert_same_run(
    circuit: &Circuit,
    steps: &[Step],
    engines: &mut [&mut dyn Simulation<Nodes>; 2],
    reported: &mut Reported,
) {
    assert_same_view(circuit, engines, reported, &"the start");
    for (index, step) in steps.iter().enumerate() {
        for engine in engines.iter_mut() {
            step.apply(&mut **engine);
        }
        assert_same_view(circuit, engines, reported, &(index, step));
    }
}

pub fn assert_fpga_runs_as_phase(name: &str, flags: &str, further: &[Step]) {
    let mut cases = CASES.iter();
    let case = cases.find(|case| case.name == name).expect("a case");
    let has = |flag: &str| flags.contains(flag);
    let circuit = match (has("--io-only"), has("--optimize")) {
        (true, _) => case.io_only_circuit(),
        (false, true) => case.optimized_circuit(),
        (false, false) => case.circuit(),
    };
    let mut phase = build(&PhaseBackend, "", &circuit);
    let mut fpga = build(&FpgaBackend, flags, &circuit);
    let reads_everything = matches!(fpga.read(), Ok(Readback::Complete(_)));
    assert_ne!(reads_everything, flags.contains("--io-only"));
    let mut engines: [&mut dyn Simulation<Nodes>; 2] = [&mut *phase, &mut *fpga];
    let shown_at_the_start = circuit.initial().states.into_iter().map(NodeState::shown);
    let shown_at_the_start: Vec<NodeState> = shown_at_the_start.collect();
    let mut reported = [shown_at_the_start.clone(), shown_at_the_start];
    if !reads_everything {
        assert_states_are_as_reported(&circuit, &mut *engines[0], &reported);
    }
    let steps = [&case.steps(&circuit), further].concat();
    assert_same_run(&circuit, &steps, &mut engines, &mut reported);

    let settled = engines.each_mut().map(|engine| engine.settle(SETTLE_LIMIT));
    let [phase_ran, fpga_ran] = settled.map(|ran| ran.expect("the engine settles"));
    match reads_everything {
        true => {
            assert_eq!(fpga_ran.stop, phase_ran.stop);
            assert!((phase_ran.ticks..=2 * phase_ran.ticks).contains(&fpga_ran.ticks));
        }
        false => assert_eq!((fpga_ran.ticks, fpga_ran.stop), (SETTLE_LIMIT, None)),
    }
    assert_same_view(&circuit, &mut engines, &mut reported, &"settling");
    if !reads_everything {
        assert_states_are_as_reported(&circuit, &mut *engines[0], &reported);
    }
}

macro_rules! comparison_tests {
    ($($name:ident($path:literal, $probe:literal, $scenario:expr) simulation[$($simulation:ident),*] board[$($board:ident),*];)*) => {
        $(mod $name {
            $(#[test]
            #[ignore = "needs Verilator"]
            fn $simulation() {
                let flags = concat!("--simulate ", crate::scenario::flags!($simulation));
                crate::comparison::assert_fpga_runs_as_phase(stringify!($name), flags, &[]);
            })*
        })*
    };
}

crate::scenario::cases!(comparison_tests);
