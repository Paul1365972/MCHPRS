use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::backend::{GraphBackend, GraphContext};
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::extract::ExtractOptions;
use mchprs_redpiler::graph::{DriveKind, Graph};
use mchprs_redpiler::nodes::{Nodes, Snapshot};
use mchprs_redpiler_phase::backend::PhaseBackend;
use mchprs_redstone::input::Input;
use mchprs_ril::Module;
use mchprs_simulation::backend::Backend;
use mchprs_simulation::readback::Readback;
use mchprs_simulation::Simulation;
use mchprs_syntax::flag::FlagValues;

pub struct Case {
    pub name: &'static str,
    ril: &'static str,
    /// `3` runs three ticks one by one and `x3` runs them at once. `lever` presses a button and
    /// toggles any other input, `plate+` and `plate-` touch and leave a pressure plate, and
    /// `knob=7` sets a strength.
    scenario: &'static str,
    probe: &'static str,
}

const REPEATER_SCENARIO: &str = "1 lever side_lever 3 lever side_lever 5 lever 5 lever 1 \
    side_lever 1 side_lever 6 lever 1 lever 12 lever 2 lever 9 lever 3 lever 9 lever 4 lever 1 \
    lever 1 lever 12 lever 1 lever 1 lever 1 lever 1 lever 1 lever 1 lever 7 lever 1 lever 3 \
    lever 1 lever 3 lever 8 lever 2 lever 1 lever 2 lever 1 lever 9 side_lever 1 lever 5 \
    side_lever 4 lever side_lever 1 side_lever 1 lever 3 side_lever 6 lever 1 side_lever 1 \
    lever 1 side_lever 8 lever_now 1 side_lever 3 lever_now 1 side_lever 4 lever_now side_lever \
    1 side_lever 2 lever_now 1 lever_now x6 lever x2 lever x3 lever x9";

macro_rules! cases {
    ($callback:ident) => {
        $callback! {
            inputs("circuits/inputs.ril", "lever_wire", "early waiting+ 2 lever 1 lever lever 2 lever 1 lever=0 3 lever=15 lit_lever \
         lit_lever=0 lit_lever=0 1 stone 3 stone 8 wooden stone 12 stone early pressed 5 wooden \
         14 stone 2 early 16 plate 4 plate 12 plate+ 11 plate- plate+ 9 plate- 10 held 9 held+ \
         8 held- 12 left+ 3 left- left 25 resting 2 resting+ 22 resting- 10 waiting- 12 waiting \
         3 waiting 12 repeater 1 repeater=0 repeater=15 locked 1 torch 3 torch=15 1 knob=3 \
         knob=15 1 knob=10 1 knob=9 1 knob=0 3 stone wooden plate+ x9 plate- x7 lever x40 stone") simulation[plain, io_only] board[plain, io_only];
            torches("circuits/torches.ril", "inverter", "2 lever 1 lever 1 lever 3 lever 2 lever lever 1 lever 1 lever 1 lever 6 lever 4 lever \
         8 lever x2 lever x5 lever x3 x17") simulation[plain, io_only] board[plain];
            repeaters("circuits/repeaters.ril", "lockable", REPEATER_SCENARIO) simulation[plain, io_only] board[plain, io_only];
            prioritized("circuits/prioritized.ril", "lockable", REPEATER_SCENARIO) simulation[plain, io_only] board[plain];
            comparators("circuits/comparators.ril", "subtract", "2 knob=15 1 knob=3 1 knob=4 1 knob=5 3 side_knob=0 3 side_knob=15 3 knob=15 3 \
         side_knob=7 3 knob=8 1 knob=6 1 knob=7 3 side_knob=6 4 knob=0 3 knob=12 side_knob=2 4 \
         bit 3 other_bit 3 bit 1 bit 1 other_bit 3 bit other_bit 3 bit 3 knob=1 3 knob=2 3 \
         bit knob=9 1 side_knob=9 1 bit 4 knob=14 other_bit x2 bit x5") simulation[plain, io_only] board[plain];
            due_together("../../../redpiler_phase/tests/circuits/due_together.ril", "compare", "1 1 2 1 1 1 2 1 2 2 2 lever 3 lever 1 1 1 3 x4") simulation[plain, io_only] board[plain];
            gate_closes("../../../redpiler_phase/tests/circuits/gate_closes.ril", "gate", "1 1 2 lever 1 1 1 2 1 2 2 2 3 1 1 1 3 lever x4") simulation[plain, io_only] board[plain];
            two_ticks_into_one_node("../../../redpiler_phase/tests/circuits/two_ticks_into_one_node.ril", "comparator", "1 1 2 1 1 1 2 lever 1 2 2 2 3 1 1 1 lever 3 x4") simulation[plain, io_only] board[plain];
            rear_and_side("../../../redpiler_phase/tests/circuits/rear_and_side.ril", "comparator", "1 1 2 1 1 1 2 1 2 lever 2 lever 2 3 1 1 1 3 x4") simulation[plain, io_only] board[plain];
            due_this_tick("../../../redpiler_phase/tests/circuits/due_this_tick.ril", "torch", "1 1 2 1 1 1 2 1 2 2 2 3 1 1 1 3 x4") simulation[plain, io_only] board[plain];
            notes("circuits/notes.ril", "clock", "lever lever lever 1 lever lever 1 sounding sounding 1 sounding 6 lever 1 lever 1 lever \
         2 lever 5 lever x13 sounding x2") simulation[plain, io_only] board[plain, io_only];
            twins("circuits/twins.ril", "first_clock", "1 lever 1 1 lever 2 lever 1 lever 3 lever x5 lever x4") simulation[plain, io_only, optimized] board[optimized];
            pulse_free("circuits/pulse_free.ril", "link", "1 lever 1 lever 2 lever lever 1 lever 3 lock_lever 2 lever 2 lock_lever 2 lever \
         lock_lever 2 lever lock_lever 2 lock_lever lever 1 lock_lever 1 lever 1 lock_lever \
         lock_lever 1 other_lever 2 lever 2 other_lever lever 2 lever other_lever 1 lever 1 \
         other_lever 3 lock_lever 1 lever 1 lever 1 lock_lever 1 lever lock_lever 1 lever \
         lock_lever lever 1 lock_lever lever lock_lever x5 lever x3 lock_lever x4") simulation[plain, io_only] board[plain];
        }
    };
}

pub(crate) use cases;

macro_rules! flags {
    (plain) => {
        ""
    };
    (io_only) => {
        "--io-only"
    };
    (optimized) => {
        "--optimize"
    };
}

pub(crate) use flags;

macro_rules! define_cases {
    ($($name:ident($path:literal, $probe:literal, $scenario:expr) simulation[$($simulation:ident),*] board[$($board:ident),*];)*) => {
        pub const CASES: &[Case] = &[$(
            Case {
                name: stringify!($name),
                ril: include_str!($path),
                scenario: $scenario,
                probe: $probe,
            },
        )*];
    };
}

cases!(define_cases);

#[derive(Debug, Clone, Copy)]
pub enum Step {
    Run(u64),
    Drive(NodeId, Input),
}

impl Step {
    pub fn apply(self, simulation: &mut dyn Simulation<Nodes>) {
        let applied = match self {
            Step::Run(ticks) => simulation.run(ticks).map(drop),
            Step::Drive(node, input) => simulation.drive(node, input).map_err(Into::into),
        };
        applied.unwrap_or_else(|error| panic!("{self:?} failed: {error:?}"));
    }
}

impl Case {
    pub fn circuit(&self) -> Circuit {
        self.observed_by(&ExtractOptions::default(), &[])
    }

    pub fn io_only_circuit(&self) -> Circuit {
        let ports_only = ExtractOptions {
            ports_only: true,
            ..ExtractOptions::default()
        };
        self.observed_by(&ports_only, &[self.probe])
    }

    pub fn optimized_circuit(&self) -> Circuit {
        let mut graph = self.graph();
        ExtractOptions::default().observe(&mut graph);
        let flags = FlagValues::parse("--optimize", PhaseBackend.flags());
        let flags = flags.expect("phase takes the flag");
        let optimized = PhaseBackend.optimize(&mut graph, &flags, &mut GraphContext::silent());
        optimized.unwrap_or_else(|error| panic!("{}: {error}", self.name));
        Circuit::new(graph).unwrap_or_else(|error| panic!("{}: {error}", self.name))
    }

    pub fn steps(&self, circuit: &Circuit) -> Vec<Step> {
        steps(self.scenario, circuit)
    }

    fn graph(&self) -> Graph {
        let module = Module::parse(self.ril);
        let module = module.unwrap_or_else(|error| panic!("{}: {error}", self.name));
        module.circuits()[0].graph()
    }

    fn observed_by(&self, options: &ExtractOptions, probes: &[&str]) -> Circuit {
        let name = self.name;
        let mut graph = self.graph();
        options.observe(&mut graph);
        for &probe in probes {
            let mut nodes = graph.node_indices();
            let probed = nodes.find(|&node| graph[node].names == [probe]);
            let probed = probed.unwrap_or_else(|| panic!("{name} has no node {probe}"));
            assert!(!graph[probed].observed, "{probe} of {name} is a port");
            graph[probed].observed = true;
        }
        Circuit::new(graph).unwrap_or_else(|error| panic!("{name}: {error}"))
    }
}

fn steps(scenario: &str, circuit: &Circuit) -> Vec<Step> {
    let steps = scenario.split_whitespace().flat_map(|step| {
        if let Ok(ticks) = step.parse() {
            return vec![Step::Run(1); ticks];
        }
        if let Some(Ok(ticks)) = step.strip_prefix('x').map(str::parse) {
            return vec![Step::Run(ticks)];
        }
        let (name, given) = match step.split_once('=') {
            Some((name, strength)) => {
                let strength: SignalStrength = strength.parse().expect("a strength");
                (name, Some(Input::Set(strength)))
            }
            None => match step.strip_suffix(['+', '-']) {
                Some(name) => (name, Some(Input::Touch(step.ends_with('+')))),
                None => (step, None),
            },
        };
        let mut nodes = circuit.ids();
        let node = nodes.find(|&node| circuit.node(node).names == [name]);
        let node = node.unwrap_or_else(|| panic!("no node {name}"));
        let input = given.unwrap_or(match circuit.node(node).kind.traits().drive {
            Some(DriveKind::Press) => Input::Press,
            _ => Input::Toggle,
        });
        let accepted = circuit.input_kinds().accept(node, input);
        accepted.unwrap_or_else(|error| panic!("{name} {error}"));
        vec![Step::Drive(node, input)]
    });
    steps.collect()
}

pub fn build(
    backend: &dyn GraphBackend,
    flags: &str,
    circuit: &Circuit,
) -> Box<dyn Simulation<Nodes>> {
    let flags = FlagValues::parse(flags, backend.flags()).expect("the backend takes the flags");
    let simulation = backend.build(circuit, &flags, &mut GraphContext::silent());
    simulation
        .unwrap_or_else(|error| panic!("{} can't build the circuit: {error:?}", backend.name()))
}

pub fn complete_snapshot(simulation: &mut dyn Simulation<Nodes>) -> Snapshot {
    let snapshot = simulation.read().expect("the engine reads its state");
    let Readback::Complete(snapshot) = snapshot else {
        panic!("the engine reads back incompletely");
    };
    snapshot
}
