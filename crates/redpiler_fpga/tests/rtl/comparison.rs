use crate::scenario::{build, complete_snapshot, Step, CASES};
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::graph::NodeState;
use mchprs_redpiler::nodes::{Changes, Nodes, Snapshot};
use mchprs_redpiler_fpga::backend::FpgaBackend;
use mchprs_redpiler_phase::backend::PhaseBackend;
use mchprs_redstone::tick_queue::PendingTick;
use mchprs_simulation::Simulation;
use std::collections::BTreeSet;
use std::fmt::Debug;

const SETTLE_LIMIT: u64 = 50;

pub type View = Vec<(NodeState, BTreeSet<PendingTick>, bool)>;

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

fn assert_same_view(
    circuit: &Circuit,
    engines: &mut [&mut dyn Simulation<Nodes>; 2],
    after: &dyn Debug,
) {
    let [phase, fpga] = engines;
    let views = view(&mut **phase).into_iter().zip(view(&mut **fpga));
    for (node, (phase, fpga)) in circuit.ids().zip(views) {
        let names = &circuit.node(node).names;
        assert_eq!(phase, fpga, "{names:?} after {after:?}");
    }
}

fn assert_same_run(
    circuit: &Circuit,
    steps: &[Step],
    engines: &mut [&mut dyn Simulation<Nodes>; 2],
) {
    assert_same_view(circuit, engines, &"the start");
    for (index, step) in steps.iter().enumerate() {
        for engine in engines.iter_mut() {
            step.apply(&mut **engine);
        }
        assert_same_view(circuit, engines, &(index, step));
    }
}

pub fn assert_fpga_runs_as_phase(name: &str, flags: &str, further: &[Step]) {
    let mut cases = CASES.iter();
    let case = cases.find(|case| case.name == name).expect("a case");
    let circuit = match flags.contains("--optimize") {
        true => case.optimized_circuit(),
        false => case.circuit(),
    };
    let mut phase = build(&PhaseBackend, "", &circuit);
    let mut fpga = build(&FpgaBackend, flags, &circuit);
    let mut engines: [&mut dyn Simulation<Nodes>; 2] = [&mut *phase, &mut *fpga];
    let steps = [&case.steps(&circuit), further].concat();
    assert_same_run(&circuit, &steps, &mut engines);

    let settled = engines.each_mut().map(|engine| engine.settle(SETTLE_LIMIT));
    let [phase_ran, fpga_ran] = settled.map(|ran| ran.expect("the engine settles"));
    assert_eq!(fpga_ran.stop, phase_ran.stop);
    assert!((phase_ran.ticks..=2 * phase_ran.ticks).contains(&fpga_ran.ticks));
    assert_same_view(&circuit, &mut engines, &"settling");
}

macro_rules! comparison_tests {
    ($($name:ident($path:literal, $scenario:expr) simulation[$($simulation:ident),*] board[$($board:ident),*];)*) => {
        $(mod $name {
            $(#[test]
            #[ignore = "needs Verilator"]
            fn $simulation() {
                let flags = crate::scenario::flags!($simulation);
                crate::comparison::assert_fpga_runs_as_phase(stringify!($name), flags, &[]);
            })*
        })*
    };
}

crate::scenario::cases!(comparison_tests);
