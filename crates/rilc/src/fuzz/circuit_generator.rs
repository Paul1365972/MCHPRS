use super::rng::Rng;
use super::scenario_generator::{random_steps, random_strength, WARM_UP_TICKS};
use crate::oracle::reference::ReferenceEngine;
use crate::oracle::{play, Step};
use crate::setup::Setup;
use mchprs_blocks::blocks::{ComparatorMode, Instrument};
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::behavior::{InputStrengths, Outcome, Pending};
use mchprs_redpiler::circuit::{inputs_in, is_stale, Circuit, NOTES, REPEATER_DELAYS};
use mchprs_redpiler::graph::{
    Graph, Link, LinkKind, Node, NodeIdx, NodeKind, NodeState, QueuedTick,
};
use mchprs_redpiler::nodes::Snapshot;
use mchprs_redstone::attached::BUTTON_PRESS_TICKS;
use mchprs_redstone::input::Input;
use mchprs_simulation::ports::Role;

const MOST_LINKS_INTO_NODE: u64 = 4;
const NODES_PER_MERGED_SHAPE: u64 = 3;
const MERGE_SHAPE_PERCENT: u64 = 50;

#[derive(Clone, Copy)]
enum Kind {
    Input,
    Repeater,
    Torch,
    Comparator,
    Constant,
    Lamp,
    Trapdoor,
    NoteBlock,
    Wire,
}

const KINDS: [Kind; 9] = [
    Kind::Input,
    Kind::Repeater,
    Kind::Torch,
    Kind::Comparator,
    Kind::Constant,
    Kind::Lamp,
    Kind::Trapdoor,
    Kind::NoteBlock,
    Kind::Wire,
];

pub struct CircuitCase {
    pub graph: Graph,
    pub circuit: Circuit,
    pub steps: Vec<Step>,
    pub observes_ports_only: bool,
}

impl CircuitCase {
    pub fn new(graph: Graph, rng: &mut Rng, ticks: u64) -> CircuitCase {
        let circuit = Circuit::new(graph.clone()).expect("a generated circuit is valid");
        CircuitCase {
            steps: random_steps(&circuit, rng, ticks),
            graph,
            circuit,
            observes_ports_only: false,
        }
    }

    pub fn setup(&self, setup: &Setup) -> String {
        match self.observes_ports_only {
            true => format!("{setup} -i"),
            false => setup.to_string(),
        }
    }
}

#[derive(Clone, Copy)]
struct Edge {
    source: NodeIdx,
    target: NodeIdx,
    link: Link,
}

pub fn random_circuit_case(rng: &mut Rng, node_count: u64, ticks: u64) -> CircuitCase {
    let mut graph = Graph::default();
    let mut edges = match rng.chance(MERGE_SHAPE_PERCENT) {
        true => merge_shape(&mut graph, rng),
        false => {
            let twin_count = rng.within(0..=node_count / NODES_PER_MERGED_SHAPE);
            add_nodes(&mut graph, rng, node_count - twin_count);
            let mut edges = random_edges(&graph, rng);
            for _ in 0..twin_count {
                add_twin(&mut graph, &mut edges, rng);
            }
            edges
        }
    };
    for _ in 0..rng.within(0..=node_count / NODES_PER_MERGED_SHAPE) {
        add_link_into_other_input(&graph, &mut edges, rng);
    }
    rng.shuffle(&mut edges);
    for edge in edges {
        graph.add_link(edge.source, edge.target, edge.link);
    }
    let observes_ports_only = rng.chance(50);
    for node in graph.node_indices().collect::<Vec<_>>() {
        graph[node].observed = graph[node].port.is_some() || !observes_ports_only;
    }
    if rng.chance(50) {
        update_each_node(&mut graph);
        graph = after_warm_up(graph, rng);
    } else {
        add_random_pending_ticks(&mut graph, rng);
        update_each_node(&mut graph);
    }
    CircuitCase {
        observes_ports_only,
        ..CircuitCase::new(graph, rng, ticks)
    }
}

fn add_nodes(graph: &mut Graph, rng: &mut Rng, count: u64) {
    let mut kinds = KINDS.to_vec();
    kinds.retain(|_| rng.chance(50));
    kinds.push(Kind::Input);
    let output_percent = rng.pick(&[0, 30, 100]);
    let first = random_kind(Kind::Input, rng);
    let state = random_state(first, rng);
    add_node(graph, first, state, Some(Role::Input));
    for _ in 1..count {
        let kind = random_kind(rng.pick(&kinds), rng);
        let state = random_state(kind, rng);
        let port = random_port(kind, output_percent, rng);
        add_node(graph, kind, state, port);
    }
}

fn add_node(graph: &mut Graph, kind: NodeKind, state: NodeState, port: Option<Role>) -> NodeIdx {
    graph.add_node(Node {
        kind,
        blocks: Default::default(),
        names: vec![format!("n{}", graph.node_count())],
        state,
        pending_tick: None,
        port,
        observed: false,
    })
}

fn random_kind(kind: Kind, rng: &mut Rng) -> NodeKind {
    let button = NodeKind::Button {
        press_ticks: rng.pick(&BUTTON_PRESS_TICKS),
    };
    match kind {
        Kind::Input => rng.pick(&[NodeKind::Lever, NodeKind::PressurePlate, button]),
        Kind::Repeater => NodeKind::Repeater {
            delay: rng.within(REPEATER_DELAYS),
            prioritized: rng.chance(20),
        },
        Kind::Torch => NodeKind::Torch,
        Kind::Comparator => NodeKind::Comparator {
            mode: rng.pick(&[ComparatorMode::Compare, ComparatorMode::Subtract]),
            far_input: rng.chance(20).then(|| random_strength(rng)),
            prioritized: rng.chance(20),
        },
        Kind::Constant => NodeKind::Constant,
        Kind::Lamp => NodeKind::Lamp,
        Kind::Trapdoor => NodeKind::Trapdoor,
        Kind::NoteBlock => NodeKind::NoteBlock {
            instrument: Instrument::Harp,
            note: rng.within(NOTES),
        },
        Kind::Wire => NodeKind::Wire,
    }
}

fn random_state(kind: NodeKind, rng: &mut Rng) -> NodeState {
    let traits = kind.traits();
    NodeState {
        power: match traits.is_on_off() {
            true => rng.pick(&[SignalStrength::Zero, SignalStrength::Fifteen]),
            false => random_strength(rng),
        },
        repeater_locked: traits.lockable && rng.chance(20),
        plate_touched: kind == NodeKind::PressurePlate && rng.chance(50),
    }
}

fn random_port(kind: NodeKind, output_percent: u64, rng: &mut Rng) -> Option<Role> {
    let traits = kind.traits();
    let input_percent = match traits.role {
        Some(Role::Input) => 90,
        _ => 10,
    };
    if traits.drive.is_some() && rng.chance(input_percent) {
        Some(Role::Input)
    } else if traits.role == Some(Role::Output) || rng.chance(output_percent) {
        Some(Role::Output)
    } else {
        None
    }
}

fn random_edges(graph: &Graph, rng: &mut Rng) -> Vec<Edge> {
    let nodes = graph.node_indices();
    let emitters: Vec<NodeIdx> = nodes
        .filter(|&node| graph[node].kind.traits().emits.is_some())
        .collect();
    let mut edges = Vec::new();
    for target in readers(graph) {
        for _ in 0..rng.within(0..=MOST_LINKS_INTO_NODE) {
            edges.push(Edge {
                source: rng.pick(&emitters),
                target,
                link: random_link(graph[target].kind, rng),
            });
        }
    }
    edges
}

fn readers(graph: &Graph) -> Vec<NodeIdx> {
    let nodes = graph.node_indices();
    nodes
        .filter(|&node| graph[node].kind.traits().reads.is_some() && !graph[node].is_input())
        .collect()
}

fn add_twin(graph: &mut Graph, edges: &mut Vec<Edge>, rng: &mut Rng) {
    let readers = readers(graph);
    if readers.is_empty() {
        return;
    }
    let original = rng.pick(&readers);
    let Node {
        kind, state, port, ..
    } = graph[original].clone();
    let twin = add_node(graph, kind, state, port);
    for edge in edges.clone() {
        if edge.target == original {
            edges.push(Edge {
                target: twin,
                ..edge
            });
        }
        if edge.source == original {
            edges.push(Edge {
                source: twin,
                link: link_into_other_input(graph[edge.target].kind, edge.link, rng),
                ..edge
            });
        }
    }
    if graph[twin].kind.traits().emits.is_some() {
        let target = rng.pick(&readers);
        edges.push(Edge {
            source: twin,
            target,
            link: random_link(graph[target].kind, rng),
        });
    }
}

fn merge_shape(graph: &mut Graph, rng: &mut Rng) -> Vec<Edge> {
    let ticking = [Kind::Torch, Kind::Repeater, Kind::Comparator];
    let driver_kind = random_kind(Kind::Input, rng);
    let driver_state = random_state(driver_kind, rng);
    let driver = add_node(graph, driver_kind, driver_state, Some(Role::Input));
    let twin_kind = random_kind(rng.pick(&ticking), rng);
    let twin_state = random_state(twin_kind, rng);
    let twin_link = random_link(twin_kind, rng);
    let mut twins = Vec::new();
    for _ in 0..rng.within(2..=3_u64) {
        twins.push(add_node(graph, twin_kind, twin_state, None));
    }
    let target_kind = random_kind(rng.pick(&ticking), rng);
    let mut targets = Vec::new();
    for _ in 0..rng.within(2..=4_u64) {
        let state = random_state(target_kind, rng);
        targets.push(add_node(graph, target_kind, state, None));
    }
    let sink_kind = random_kind(rng.pick(&[Kind::Lamp, Kind::Wire, Kind::Trapdoor]), rng);
    let sink_state = random_state(sink_kind, rng);
    let sink = add_node(graph, sink_kind, sink_state, Some(Role::Output));
    let mut edges = Vec::new();
    for &twin in &twins {
        edges.push(Edge {
            source: driver,
            target: twin,
            link: twin_link,
        });
        for &target in &targets {
            let feeds_both_inputs = target_kind.traits().side_inputs && rng.chance(30);
            if !feeds_both_inputs && !rng.chance(60) {
                continue;
            }
            let link = random_link(target_kind, rng);
            edges.push(Edge {
                source: twin,
                target,
                link,
            });
            if feeds_both_inputs {
                edges.push(Edge {
                    source: twin,
                    target,
                    link: link_into_other_input(target_kind, link, rng),
                });
            }
        }
    }
    for &source in &targets {
        for &target in &targets {
            if source != target && rng.chance(40) {
                let link = random_link(target_kind, rng);
                edges.push(Edge {
                    source,
                    target,
                    link,
                });
            }
        }
        if rng.chance(70) {
            edges.push(Edge {
                source,
                target: sink,
                link: Link::new(LinkKind::Default, 0),
            });
        }
    }
    edges
}

fn add_link_into_other_input(graph: &Graph, edges: &mut Vec<Edge>, rng: &mut Rng) {
    if edges.is_empty() {
        return;
    }
    let edge = rng.pick(edges);
    edges.push(Edge {
        link: link_into_other_input(graph[edge.target].kind, edge.link, rng),
        ..edge
    });
}

fn random_link(target: NodeKind, rng: &mut Rng) -> Link {
    let kind = match target.traits().side_inputs && rng.chance(30) {
        true => LinkKind::Side,
        false => LinkKind::Default,
    };
    Link::new(kind, random_weight(rng))
}

fn link_into_other_input(target: NodeKind, link: Link, rng: &mut Rng) -> Link {
    let kind = match (target.traits().side_inputs, link.kind) {
        (true, LinkKind::Default) => LinkKind::Side,
        _ => LinkKind::Default,
    };
    Link::new(kind, random_weight(rng))
}

fn random_weight(rng: &mut Rng) -> u8 {
    let weights = rng.pick(&[0..=0, 0..=5, 6..=14]);
    rng.within(weights)
}

fn update_each_node(graph: &mut Graph) {
    let nodes: Vec<NodeIdx> = graph.node_indices().collect();
    for (index, &node) in nodes.iter().enumerate() {
        let inputs = inputs_in(graph, node);
        let node = &mut graph[node];
        let pending = node.pending();
        let outcome = match node.is_input() {
            true if is_stale(node, inputs) => Outcome {
                schedule: node.kind.schedulable_ticks().first().copied(),
                ..Outcome::of(node.state)
            },
            true => continue,
            false => node.kind.update(node.state, inputs, pending),
        };
        node.state = outcome.state;
        if let Some(tick) = outcome.schedule {
            let order = nodes.len() + index;
            node.pending_tick = Some(QueuedTick { tick, order });
        }
    }
}

fn after_warm_up(graph: Graph, rng: &mut Rng) -> Graph {
    let circuit = Circuit::new(graph.clone()).expect("a settled circuit is valid");
    let mut engine = ReferenceEngine::new(&circuit);
    let ticks = rng.within(0..=WARM_UP_TICKS);
    for step in random_steps(&circuit, rng, ticks) {
        let played = play(&mut engine, step);
        played.expect("the reference engine never fails");
    }
    at_snapshot(&graph, &engine.complete_snapshot())
}

fn at_snapshot(graph: &Graph, snapshot: &Snapshot) -> Graph {
    let mut graph = graph.clone();
    let nodes: Vec<NodeIdx> = graph.node_indices().collect();
    for (&node, &state) in nodes.iter().zip(&snapshot.states) {
        graph[node].state = state;
        graph[node].pending_tick = None;
    }
    for (order, &(id, tick)) in snapshot.pending_ticks.iter().enumerate() {
        graph[nodes[id.index()]].pending_tick = Some(QueuedTick { tick, order });
    }
    graph
}

fn add_random_pending_ticks(graph: &mut Graph, rng: &mut Rng) {
    let mut scheduling = Vec::new();
    for node in graph.node_indices() {
        if !graph[node].can_tick() || !rng.chance(25) {
            continue;
        }
        if let Some(outcome) = random_scheduling_outcome(&graph[node], rng) {
            scheduling.push((node, outcome));
        }
    }
    let mut orders: Vec<usize> = (0..scheduling.len()).collect();
    rng.shuffle(&mut orders);
    for ((node, outcome), order) in scheduling.into_iter().zip(orders) {
        graph[node].state = outcome.state;
        graph[node].pending_tick = outcome.schedule.map(|tick| QueuedTick { tick, order });
    }
}

fn random_scheduling_outcome(node: &Node, rng: &mut Rng) -> Option<Outcome> {
    let mut outcomes = Vec::new();
    for default in SignalStrength::all() {
        let inputs = InputStrengths {
            default,
            side: SignalStrength::Zero,
        };
        outcomes.push(node.kind.update(node.state, inputs, Pending::None));
        outcomes.push(node.kind.tick(node.state, inputs));
    }
    let press = node.kind.accept(Input::Press);
    outcomes.extend(press.map(|press| press.drive(node.state, Pending::None)));
    outcomes.retain(|outcome| outcome.schedule.is_some());
    if outcomes.is_empty() {
        return None;
    }
    let mut outcome = rng.pick(&outcomes);
    if let Some(tick) = &mut outcome.schedule {
        tick.delay = rng.within(1..=tick.delay);
    }
    Some(outcome)
}
