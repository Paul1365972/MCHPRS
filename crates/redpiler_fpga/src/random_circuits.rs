use crate::circuit::{
    Circuit, ComparatorMode, Kind, Link, Node, PendingTick, State, PULSE_PRIORITY,
};
use crate::engine::Simulator;
use crate::reference::{settled_initial_state, Reference};
use crate::verilator::Verilator;
use mchprs_redpiler::compile_graph::SignalStrength;
use std::num::NonZeroU8;

const LINK_WINDOW: usize = 24;

struct Random(u64);

impl Random {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    fn below(&mut self, bound: usize) -> usize {
        (self.next() % bound as u64) as usize
    }

    fn chance(&mut self, numerator: usize, denominator: usize) -> bool {
        self.below(denominator) < numerator
    }

    fn strength(&mut self) -> SignalStrength {
        SignalStrength::try_from(self.below(16) as u8).expect("strengths are below 16")
    }
}

fn random_kind(random: &mut Random) -> Kind {
    match random.below(20) {
        0 => Kind::Lever,
        1 => Kind::Button,
        2 => Kind::PressurePlate,
        3 => Kind::Constant,
        4..=6 => Kind::Torch,
        7..=10 => Kind::Repeater {
            delay: 1 + random.below(4) as u8,
            facing_diode: random.chance(1, 4),
        },
        11..=13 => Kind::Comparator {
            mode: if random.chance(1, 2) {
                ComparatorMode::Subtract
            } else {
                ComparatorMode::Compare
            },
            far_input: random.chance(1, 5).then(|| random.strength()),
            facing_diode: random.chance(1, 4),
        },
        14 | 15 => Kind::Lamp,
        16 | 17 => Kind::Trapdoor,
        _ => Kind::Wire,
    }
}

fn random_state(random: &mut Random, kind: Kind, has_sides: bool) -> State {
    let mut state = State::default();
    match kind {
        Kind::Constant | Kind::Wire => state.strength = random.strength(),
        Kind::Comparator { .. } => {
            if random.chance(1, 2) {
                state.strength = random.strength();
            }
        }
        _ => {
            state.set_powered(random.chance(1, 2));
            state.locked =
                matches!(kind, Kind::Repeater { .. }) && has_sides && random.chance(1, 4);
        }
    }
    if let Some(scheduled) = kind
        .scheduled_tick(state.is_powered())
        .filter(|_| random.chance(1, 4))
    {
        let delay = random.below(usize::from(scheduled.delay.get())) as u8;
        let priority = if kind.has_pulse_countdown() && random.chance(1, 2) {
            PULSE_PRIORITY
        } else {
            scheduled.priority
        };
        state.pending = Some(PendingTick {
            delay: NonZeroU8::MIN.saturating_add(delay),
            priority,
        });
    }
    state
}

fn random_links(random: &mut Random, kinds: &[Kind], target: usize, count: usize) -> Vec<Link> {
    let low = target.saturating_sub(LINK_WINDOW);
    let high = (target + LINK_WINDOW).min(kinds.len() - 1);
    (0..count)
        .filter_map(|_| {
            let source = low + random.below(high - low + 1);
            let attenuation = match random.below(10) {
                0..=4 => 0,
                5..=7 => 1 + random.below(3) as u8,
                _ => random.below(16) as u8,
            };
            (kinds[source].drives_links() && kinds[source] != Kind::Wire).then_some(Link {
                source,
                attenuation,
            })
        })
        .collect()
}

fn random_circuit(seed: u64, count: usize) -> Circuit {
    let mut random = Random(seed.wrapping_mul(0x9e3779b97f4a7c15) | 1);
    let kinds: Vec<_> = (0..count).map(|_| random_kind(&mut random)).collect();
    let mut nodes: Vec<_> = kinds
        .iter()
        .enumerate()
        .map(|(index, &kind)| {
            let (inputs, sides) = match kind {
                Kind::Lever | Kind::Button | Kind::PressurePlate | Kind::Constant => (0, 0),
                Kind::Repeater { .. } | Kind::Comparator { .. } => {
                    (1 + random.below(3), random.below(3))
                }
                Kind::Wire => (1 + random.below(5), 0),
                _ => (1 + random.below(3), 0),
            };
            let inputs = random_links(&mut random, &kinds, index, inputs);
            let sides = random_links(&mut random, &kinds, index, sides);
            Node {
                kind,
                initial: random_state(&mut random, kind, !sides.is_empty()),
                inputs,
                sides,
                is_input: kind.is_input(),
                is_output: matches!(kind, Kind::Lamp | Kind::Trapdoor),
            }
        })
        .collect();
    let settled: Vec<_> = (0..nodes.len())
        .map(|index| settled_initial_state(&nodes, index))
        .collect();
    for (node, state) in nodes.iter_mut().zip(settled) {
        node.initial = state;
    }
    Circuit::new(nodes).expect("random circuits are valid")
}

fn assert_same_states(
    circuit: &Circuit,
    reference: &mut Reference,
    simulator: &mut dyn Simulator,
    seed: u64,
    moment: &str,
) {
    let expected = reference.states().unwrap();
    let actual = simulator.states().unwrap();
    if let Some(index) = (0..expected.len()).find(|&index| expected[index] != actual[index]) {
        let node = &circuit.nodes()[index];
        panic!(
            "seed {seed}, {moment}: node {index} ({:?}, inputs {:?}, sides {:?}, initial {:?}) is {:?} in the reference and {:?} in the circuit",
            node.kind, node.inputs, node.sides, node.initial, expected[index], actual[index]
        );
    }
}

fn input_nodes(circuit: &Circuit) -> Vec<usize> {
    (0..circuit.nodes().len())
        .filter(|&index| circuit.nodes()[index].kind.is_input())
        .collect()
}

fn simulated_circuit(seed: u64, count: usize, ticks: u64) -> Circuit {
    let circuit = random_circuit(seed, count);
    let mut reference = Reference::new(&circuit);
    let inputs = input_nodes(&circuit);
    let mut random = Random(seed ^ 0x2545f4914f6cdd1d);
    for _ in 0..ticks {
        if !inputs.is_empty() && random.chance(1, 3) {
            let node = inputs[random.below(inputs.len())];
            reference.input(node, random.chance(1, 2)).unwrap();
        }
        reference.advance(1).unwrap();
    }
    let nodes = circuit
        .nodes()
        .iter()
        .zip(reference.states().unwrap())
        .map(|(node, initial)| Node {
            initial,
            ..node.clone()
        })
        .collect();
    Circuit::new(nodes).expect("simulated circuits are valid")
}

fn compare_with_reference(
    seed: u64,
    circuit: &Circuit,
    ticks: u64,
    simulator: impl FnOnce(&Circuit) -> Box<dyn Simulator>,
) {
    let mut reference = Reference::new(circuit);
    let mut simulator = simulator(circuit);
    let inputs = input_nodes(circuit);
    let mut random = Random(seed ^ 0x5851f42d4c957f2d);
    assert_same_states(
        circuit,
        &mut reference,
        simulator.as_mut(),
        seed,
        "initially",
    );
    for tick in 0..ticks {
        if !inputs.is_empty() && random.chance(1, 3) {
            let node = inputs[random.below(inputs.len())];
            let powered = random.chance(1, 2);
            reference.input(node, powered).unwrap();
            simulator.input(node, powered).unwrap();
            let moment = format!("after input {powered} to node {node} at tick {tick}");
            assert_same_states(circuit, &mut reference, simulator.as_mut(), seed, &moment);
        }
        reference.advance(1).unwrap();
        simulator.advance(1).unwrap();
        let moment = format!("at tick {}", tick + 1);
        assert_same_states(circuit, &mut reference, simulator.as_mut(), seed, &moment);
    }
}

#[test]
fn random_circuits_are_valid() {
    for seed in 0..100 {
        random_circuit(seed, 500);
    }
}

#[test]
#[ignore = "needs Verilator"]
fn random_circuits_match_the_reference_in_verilator() {
    let verilator = |circuit: &Circuit| -> Box<dyn Simulator> {
        Box::new(Verilator::compile(circuit, &|_| {}).unwrap())
    };
    for seed in 0..8 {
        compare_with_reference(seed, &random_circuit(seed, 1500), 2000, verilator);
    }
    for seed in 8..12 {
        compare_with_reference(seed, &simulated_circuit(seed, 1500, 200), 2000, verilator);
    }
}
