use super::{update, DirectEngine};
use crate::node::{NodeInput, Slot};
use crate::scheduler::TickScheduler;
use mchprs_blocks::blocks::ComparatorMode;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::behavior::{InputStrengths, Outcome, Pending};
use mchprs_redpiler::circuit::Circuit;
use mchprs_redpiler::graph::{Graph, Node, NodeKind, NodeState, QueuedTick};
use mchprs_redstone::tick_queue::PendingTick;
use std::iter;

fn hand_written_kinds() -> Vec<NodeKind> {
    let mut kinds = vec![NodeKind::Torch];
    for delay in 1..=4 {
        for prioritized in [false, true] {
            kinds.push(NodeKind::Repeater { delay, prioritized });
        }
    }
    for mode in [ComparatorMode::Compare, ComparatorMode::Subtract] {
        for far_input in iter::once(None).chain(SignalStrength::all().map(Some)) {
            for prioritized in [false, true] {
                kinds.push(NodeKind::Comparator {
                    mode,
                    far_input,
                    prioritized,
                });
            }
        }
    }
    kinds
}

fn states(kind: NodeKind) -> Vec<NodeState> {
    match kind {
        NodeKind::Repeater { .. } => [false, true]
            .into_iter()
            .flat_map(|powered| [false, true].map(|locked| NodeState::repeater(powered, locked)))
            .collect(),
        NodeKind::Torch => [false, true].map(NodeState::from_powered).to_vec(),
        _ => SignalStrength::all().map(NodeState::from_power).collect(),
    }
}

fn inputs(kind: NodeKind) -> Vec<InputStrengths> {
    let sides = if kind.traits().side_inputs { 16 } else { 1 };
    let strengths = SignalStrength::all();
    let inputs = strengths.flat_map(|default| {
        let sides = SignalStrength::all().take(sides);
        sides.map(move |side| InputStrengths { default, side })
    });
    inputs.collect()
}

fn engine_of(kind: NodeKind) -> DirectEngine {
    let unpowered = InputStrengths::default();
    let state = NodeState::default();
    let settling_tick = kind.update(state, unpowered, Pending::None).schedule;
    let mut graph = Graph::default();
    graph.add_node(Node {
        kind,
        blocks: Default::default(),
        names: Vec::new(),
        state,
        pending_tick: settling_tick.map(|tick| QueuedTick { tick, order: 0 }),
        port: None,
        observed: true,
    });
    let circuit = Circuit::new(graph).expect("one node is a circuit");
    DirectEngine::new(&circuit).expect("Direct runs the node")
}

fn input(power: SignalStrength) -> NodeInput {
    let mut input = NodeInput::new();
    input.update_power(SignalStrength::Zero, power);
    input
}

fn set_up(
    engine: &mut DirectEngine,
    state: NodeState,
    inputs: InputStrengths,
    pending: Pending,
) -> Slot {
    engine.scheduler = TickScheduler::default();
    let tick = engine.scheduler.current_tick();
    let slot = engine.nodes.get(0);
    let node = &mut engine.nodes[slot];
    node.state.power = state.power;
    node.state.repeater_locked = state.repeater_locked;
    node.default_inputs = input(inputs.default);
    node.side_inputs = input(inputs.side);
    node.due_tick = match pending {
        Pending::None => None,
        Pending::DueThisTick => Some(tick),
        Pending::Later => Some(tick + 1),
    };
    slot
}

fn result(engine: &DirectEngine, slot: Slot) -> (NodeState, Vec<PendingTick>) {
    let scheduled = engine.scheduler.pending_ticks().map(|(_, tick)| tick);
    (engine.nodes[slot].state, scheduled.collect())
}

fn expected(outcome: Outcome) -> (NodeState, Vec<PendingTick>) {
    (outcome.state, outcome.schedule.into_iter().collect())
}

#[test]
fn repeaters_torches_and_comparators_update_and_tick_as_the_spec_says() {
    for kind in hand_written_kinds() {
        let mut engine = engine_of(kind);
        for state in states(kind) {
            for inputs in inputs(kind) {
                for pending in Pending::ALL {
                    let slot = set_up(&mut engine, state, inputs, pending);
                    update::update_node(&mut engine.scheduler, &mut engine.nodes, slot);
                    let spec = expected(kind.update(state, inputs, pending));
                    let case = format!("{kind:?} in {state:?} with {inputs:?}, {pending:?}");
                    assert_eq!(result(&engine, slot), spec, "{case}");
                }
                let slot = set_up(&mut engine, state, inputs, Pending::DueThisTick);
                engine.tick_node(slot);
                let spec = expected(kind.tick(state, inputs));
                let case = format!("tick of {kind:?} in {state:?} with {inputs:?}");
                assert_eq!(result(&engine, slot), spec, "{case}");
            }
        }
    }
}
