use crate::graph::{DriveKind, LinkKind, Node, NodeKind, NodeState};
use itertools::iproduct;
use mchprs_blocks::blocks::ComparatorMode;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redstone::input::Input;
use mchprs_redstone::tick_queue::{PendingTick, TickPriority};
use mchprs_redstone::{attached, comparator, plate, repeater, torch, LAMP_OFF_DELAY_TICKS};

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct InputStrengths {
    pub default: SignalStrength,
    pub side: SignalStrength,
}

impl InputStrengths {
    pub fn raise(&mut self, kind: LinkKind, strength: SignalStrength) {
        let input = match kind {
            LinkKind::Default => &mut self.default,
            LinkKind::Side => &mut self.side,
        };
        *input = strength.max(*input);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pending {
    None,
    DueThisTick,
    Later,
}

impl Pending {
    pub const ALL: [Pending; 3] = [Pending::None, Pending::DueThisTick, Pending::Later];
}

impl Node {
    pub fn pending(&self) -> Pending {
        match self.pending_tick {
            Some(_) => Pending::Later,
            None => Pending::None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Outcome {
    pub state: NodeState,
    pub schedule: Option<PendingTick>,
    pub plays_note: bool,
}

impl Outcome {
    pub fn of(state: NodeState) -> Outcome {
        Outcome {
            state,
            schedule: None,
            plays_note: false,
        }
    }

    fn scheduled(state: NodeState, delay: u32, priority: TickPriority) -> Outcome {
        Outcome {
            schedule: Some(PendingTick { delay, priority }),
            ..Outcome::of(state)
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AcceptedInput {
    Set(SignalStrength),
    Toggle,
    Press { press_ticks: u8 },
    Touch(bool),
    ToggleTouch,
}

impl NodeKind {
    pub fn update(self, state: NodeState, inputs: InputStrengths, pending: Pending) -> Outcome {
        let powered = !inputs.default.is_zero();
        match self {
            NodeKind::Repeater { delay, prioritized } => {
                let state = NodeState {
                    repeater_locked: !inputs.side.is_zero(),
                    ..state
                };
                if state.repeater_locked
                    || pending != Pending::None
                    || powered == state.is_powered()
                {
                    return Outcome::of(state);
                }
                let priority = repeater::tick_priority(prioritized, state.is_powered());
                Outcome::scheduled(state, delay.into(), priority)
            }
            NodeKind::Torch if pending == Pending::None && powered == state.is_powered() => {
                Outcome::scheduled(state, torch::DELAY_TICKS, torch::PRIORITY)
            }
            NodeKind::Comparator {
                mode,
                far_input,
                prioritized,
            } if pending == Pending::None
                && comparator_output(mode, far_input, inputs) != state.power =>
            {
                let priority = comparator::tick_priority(prioritized);
                Outcome::scheduled(state, comparator::DELAY_TICKS, priority)
            }
            NodeKind::Lamp => match (state.is_powered(), powered) {
                (false, true) => Outcome::of(NodeState::from_powered(true)),
                (true, false) if pending != Pending::Later => {
                    Outcome::scheduled(state, LAMP_OFF_DELAY_TICKS, TickPriority::Normal)
                }
                _ => Outcome::of(state),
            },
            NodeKind::Trapdoor => Outcome::of(NodeState::from_powered(powered)),
            NodeKind::Wire => Outcome::of(NodeState::from_power(inputs.default)),
            NodeKind::NoteBlock { .. } => Outcome {
                plays_note: powered && !state.is_powered(),
                ..Outcome::of(NodeState::from_powered(powered))
            },
            NodeKind::Torch
            | NodeKind::Comparator { .. }
            | NodeKind::Button { .. }
            | NodeKind::Lever
            | NodeKind::PressurePlate
            | NodeKind::Constant => Outcome::of(state),
        }
    }

    pub fn tick(self, state: NodeState, inputs: InputStrengths) -> Outcome {
        let powered = !inputs.default.is_zero();
        match self {
            NodeKind::Repeater { delay, .. } if !state.repeater_locked => {
                let on = NodeState {
                    power: SignalStrength::Fifteen,
                    ..state
                };
                match (state.is_powered(), powered) {
                    (true, false) => Outcome::of(NodeState {
                        power: SignalStrength::Zero,
                        ..state
                    }),
                    (false, false) => Outcome::scheduled(on, delay.into(), TickPriority::Higher),
                    (false, true) => Outcome::of(on),
                    (true, true) => Outcome::of(state),
                }
            }
            NodeKind::Torch => Outcome::of(NodeState::from_powered(!powered)),
            NodeKind::Comparator {
                mode, far_input, ..
            } => Outcome::of(NodeState::from_power(comparator_output(
                mode, far_input, inputs,
            ))),
            NodeKind::Lamp if !powered => Outcome::of(NodeState::from_powered(false)),
            NodeKind::Button { .. } => Outcome::of(NodeState::from_powered(false)),
            NodeKind::PressurePlate if state.is_powered() && state.plate_touched => {
                Outcome::scheduled(state, plate::PRESS_TICKS, plate::RECHECK_PRIORITY)
            }
            NodeKind::PressurePlate => Outcome::of(NodeState {
                power: SignalStrength::Zero,
                ..state
            }),
            NodeKind::Repeater { .. }
            | NodeKind::Lamp
            | NodeKind::Lever
            | NodeKind::Trapdoor
            | NodeKind::Wire
            | NodeKind::Constant
            | NodeKind::NoteBlock { .. } => Outcome::of(state),
        }
    }

    pub fn accept(self, input: Input) -> Option<AcceptedInput> {
        use SignalStrength::{Fifteen, Zero};
        Some(match (self.traits().drive?, self, input) {
            (DriveKind::OnOff, _, Input::Toggle) => AcceptedInput::Toggle,
            (DriveKind::OnOff, _, Input::Set(power @ (Zero | Fifteen)))
            | (DriveKind::Strength, _, Input::Set(power)) => AcceptedInput::Set(power),
            (DriveKind::Press, NodeKind::Button { press_ticks }, Input::Press) => {
                AcceptedInput::Press { press_ticks }
            }
            (DriveKind::Touch, _, Input::Toggle) => AcceptedInput::ToggleTouch,
            (DriveKind::Touch, _, Input::Touch(touched)) => AcceptedInput::Touch(touched),
            _ => return None,
        })
    }

    pub fn schedulable_ticks(self) -> Vec<PendingTick> {
        let strengths: Vec<SignalStrength> = SignalStrength::all().collect();
        let flags = [false, true];
        let accepted_inputs: Vec<AcceptedInput> = Input::all()
            .filter_map(|input| self.accept(input))
            .collect();
        let mut ticks = Vec::new();
        let mut collect = |outcome: Outcome| {
            if let Some(tick) = outcome.schedule
                && !ticks.contains(&tick)
            {
                ticks.push(tick);
            }
        };
        for (&power, repeater_locked, plate_touched) in iproduct!(&strengths, flags, flags) {
            let state = NodeState {
                power,
                repeater_locked,
                plate_touched,
            };
            for (&default, &side) in iproduct!(&strengths, &strengths) {
                let inputs = InputStrengths { default, side };
                collect(self.tick(state, inputs));
                for pending in Pending::ALL {
                    collect(self.update(state, inputs, pending));
                }
            }
            for (&input, pending) in iproduct!(&accepted_inputs, Pending::ALL) {
                collect(input.drive(state, pending));
            }
        }
        ticks
    }
}

impl AcceptedInput {
    pub fn drive(self, state: NodeState, pending: Pending) -> Outcome {
        let with_power = |power| NodeState { power, ..state };
        match self {
            AcceptedInput::Set(power) => Outcome::of(with_power(power)),
            AcceptedInput::Toggle => Outcome::of(with_power((!state.is_powered()).into())),
            AcceptedInput::Press { press_ticks } => {
                let pressed = with_power(SignalStrength::Fifteen);
                match (state.is_powered(), pending) {
                    (true, _) => Outcome::of(state),
                    (false, Pending::None) => {
                        Outcome::scheduled(pressed, press_ticks.into(), attached::BUTTON_PRIORITY)
                    }
                    (false, Pending::DueThisTick | Pending::Later) => Outcome::of(pressed),
                }
            }
            AcceptedInput::Touch(touched) => touch(state, touched, pending),
            AcceptedInput::ToggleTouch => touch(state, !state.plate_touched, pending),
        }
    }
}

fn touch(state: NodeState, plate_touched: bool, pending: Pending) -> Outcome {
    let state = NodeState {
        plate_touched,
        ..state
    };
    if !plate_touched || state.is_powered() {
        return Outcome::of(state);
    }
    let pressed = NodeState {
        power: SignalStrength::Fifteen,
        ..state
    };
    match pending {
        Pending::None => Outcome::scheduled(pressed, plate::PRESS_TICKS, plate::RECHECK_PRIORITY),
        Pending::DueThisTick | Pending::Later => Outcome::of(pressed),
    }
}

#[inline]
pub fn comparator_output(
    mode: ComparatorMode,
    far_input: Option<SignalStrength>,
    inputs: InputStrengths,
) -> SignalStrength {
    let rear = match far_input {
        Some(far_input) if inputs.default < SignalStrength::Fifteen => far_input,
        _ => inputs.default,
    };
    mode.output(rear.into(), inputs.side.into()).capped()
}
