use crate::circuit::{initial_input_strength, Kind, Node, PendingTick, State};
use mchprs_redpiler::compile_graph::SignalStrength;

pub(super) fn settle_node(
    kind: Kind,
    mut state: State,
    input: SignalStrength,
    side: SignalStrength,
    pending: bool,
) -> (State, Option<PendingTick>) {
    let input_powered = !input.is_zero();
    let changes = match kind {
        Kind::Wire => {
            state.strength = input;
            false
        }
        Kind::Repeater { .. } => {
            state.locked = !side.is_zero();
            !state.locked && state.is_powered() != input_powered
        }
        Kind::Torch => state.is_powered() == input_powered,
        Kind::Comparator {
            mode, far_input, ..
        } => mode.output(far_input, input, side) != state.strength,
        Kind::Lamp => {
            if !state.is_powered() && input_powered {
                state.set_powered(true);
            }
            state.is_powered() && !input_powered
        }
        Kind::Trapdoor => {
            state.set_powered(input_powered);
            false
        }
        _ => false,
    };
    let tick = kind
        .scheduled_tick(state.is_powered())
        .filter(|_| changes && !pending);
    (state, tick)
}

pub(crate) fn settled_initial_state(nodes: &[Node], index: usize) -> State {
    let node = &nodes[index];
    let (mut state, tick) = settle_node(
        node.kind,
        node.initial,
        initial_input_strength(nodes, &node.inputs),
        initial_input_strength(nodes, &node.sides),
        node.initial.pending.is_some(),
    );
    state.pending = tick.or(state.pending);
    state
}
