use crate::circuit::{Circuit, ComparatorMode, Kind, Link, State};

pub(super) fn comparator(mode: ComparatorMode, far_input: Option<u8>, input: u8, side: u8) -> u8 {
    let rear = if input < 15 {
        far_input.unwrap_or(input)
    } else {
        input
    };
    match mode {
        ComparatorMode::Compare if rear >= side => rear,
        ComparatorMode::Compare => 0,
        ComparatorMode::Subtract => rear.saturating_sub(side),
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Schedule {
    pub(super) delay: u8,
    pub(super) priority: u8,
}

pub(super) fn settle_node(
    kind: Kind,
    mut state: State,
    input: u8,
    side: u8,
    pending: bool,
) -> (State, Option<Schedule>) {
    let schedule = |delay, priority| Some(Schedule { delay, priority });
    let schedule = match kind {
        Kind::Wire => {
            state.strength = input;
            None
        }
        Kind::Repeater {
            delay,
            facing_diode,
        } => {
            state.locked = side != 0;
            if !state.locked && !pending && state.is_powered() != (input != 0) {
                let priority = if facing_diode {
                    0
                } else if input == 0 {
                    1
                } else {
                    2
                };
                schedule(delay, priority)
            } else {
                None
            }
        }
        Kind::Torch if !pending && state.is_powered() != (input == 0) => schedule(1, 3),
        Kind::Comparator {
            mode,
            far_input,
            facing_diode,
        } if !pending && comparator(mode, far_input, input, side) != state.strength => {
            schedule(1, if facing_diode { 2 } else { 3 })
        }
        Kind::Lamp if state.is_powered() && input == 0 && !pending => schedule(2, 3),
        Kind::Lamp if !state.is_powered() && input != 0 => {
            state.set_powered(true);
            None
        }
        Kind::Trapdoor if state.is_powered() != (input != 0) => {
            state.set_powered(input != 0);
            None
        }
        _ => None,
    };
    (state, schedule)
}

/// Settling a node whose inputs did not change leaves a settled node unchanged, and firing leaves
/// every node settled, so a node that starts settled can be settled on every clock instead of only
/// when an input changes.
pub(crate) fn is_initially_settled(circuit: &Circuit, index: usize) -> bool {
    let nodes = circuit.nodes();
    let node = &nodes[index];
    let strongest = |links: &[Link]| {
        links
            .iter()
            .map(|link| {
                nodes[link.source]
                    .initial
                    .strength
                    .saturating_sub(link.attenuation)
            })
            .max()
            .unwrap_or(0)
    };
    let pending = node.kind.is_timed() && node.initial.delay != 0;
    let settled = settle_node(
        node.kind,
        node.initial,
        strongest(&node.inputs),
        strongest(&node.sides),
        pending,
    );
    settled == (node.initial, None)
}
