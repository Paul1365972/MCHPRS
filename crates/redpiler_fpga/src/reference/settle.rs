use crate::circuit::{ComparatorMode, Kind, Link, Node, State};

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

pub(crate) fn settled_initial_state(nodes: &[Node], index: usize) -> State {
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
    let (mut state, schedule) = settle_node(
        node.kind,
        node.initial,
        strongest(&node.inputs),
        strongest(&node.sides),
        pending,
    );
    if let Some(Schedule { delay, priority }) = schedule {
        state.delay = delay;
        state.priority = priority;
    }
    state
}
