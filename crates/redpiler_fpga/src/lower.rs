mod names;

use names::{FiringPort, NodeModule, Parameter};

use crate::protocol::DRIVE_NODE_BITS;
use mchprs_blocks::blocks::ComparatorMode;
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::graph::{LinkKind, Node, NodeKind, SignalKind};
use mchprs_redstone::tick_queue::TickPriority;
use mchprs_redstone::{attached, comparator, plate, repeater, torch, LAMP_OFF_DELAY_TICKS};
use mchprs_simulation::backend::CompileError;
use std::iter;

/// The priority in whose clock a redstone tick starts, which prioritized repeaters fire in.
const TICK_START_PRIORITY: TickPriority = TickPriority::Highest;
/// The priority that vanilla gives the tick that ends a repeater's pulse.
const PULSE_END_PRIORITY: TickPriority = TickPriority::Higher;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CircuitSignal {
    TakenValid,
    TakenNode,
    TakenInput,
    ShownRead,
}

impl CircuitSignal {
    pub const ALL: [Self; 4] = [
        Self::TakenValid,
        Self::TakenNode,
        Self::TakenInput,
        Self::ShownRead,
    ];
    const DRIVE: &'static [Self] = &[Self::TakenValid, Self::TakenNode, Self::TakenInput];
    const PRESS: &'static [Self] = &[Self::TakenValid, Self::TakenNode];

    pub fn name(self) -> &'static str {
        match self {
            Self::TakenValid => "taken_valid",
            Self::TakenNode => "taken_node",
            Self::TakenInput => "taken_input",
            Self::ShownRead => "shown_read",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Register {
    Powered,
    Strength,
    Locked,
    Played,
    Touched,
    Countdown { delay: u32, priority: PriorityRule },
    Pulsing,
}

impl Register {
    pub fn port(self) -> &'static str {
        match self {
            Register::Powered => "powered",
            Register::Strength => "strength",
            Register::Locked => "locked",
            Register::Played => "played",
            Register::Touched => "touched",
            Register::Countdown { .. } => "countdown",
            Register::Pulsing => "pulsing",
        }
    }

    pub fn width(self) -> usize {
        match self {
            Register::Strength => 4,
            Register::Countdown { delay, .. } => delay as usize,
            _ => 1,
        }
    }

    pub fn is_shown(self) -> bool {
        !matches!(
            self,
            Register::Touched | Register::Countdown { .. } | Register::Pulsing
        )
    }

    fn parameters(self, node: &Node) -> Vec<(Parameter, u32)> {
        let power = node.state.power.get().into();
        let pending = node.pending_tick.map(|queued| queued.tick);
        match self {
            Register::Powered => vec![(Parameter::Powered, node.state.is_powered().into())],
            Register::Strength => vec![(Parameter::Strength, power)],
            Register::Locked => vec![(Parameter::Locked, node.state.repeater_locked.into())],
            Register::Played => Vec::new(),
            Register::Touched => vec![(Parameter::Touched, node.state.plate_touched.into())],
            Register::Countdown { delay, .. } => vec![
                (Parameter::Delay, delay),
                (
                    Parameter::PendingDelay,
                    pending.map_or(0, |tick| tick.delay),
                ),
            ],
            Register::Pulsing => {
                let pulsing = pending.is_some_and(|tick| tick.priority == PULSE_END_PRIORITY);
                vec![(Parameter::Pulsing, pulsing.into())]
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PriorityRule {
    Fixed(TickPriority),
    ByOutput { off: TickPriority, on: TickPriority },
    ByPulsing,
}

impl PriorityRule {
    pub fn of(self, powered: bool, pulsing: bool) -> TickPriority {
        match self {
            PriorityRule::Fixed(priority) => priority,
            PriorityRule::ByOutput { off, on } => {
                if powered {
                    on
                } else {
                    off
                }
            }
            PriorityRule::ByPulsing if pulsing => PULSE_END_PRIORITY,
            PriorityRule::ByPulsing => TICK_START_PRIORITY,
        }
    }

    fn firing_ports(self) -> Vec<(FiringPort, TickPriority)> {
        match self {
            PriorityRule::Fixed(priority) => vec![(FiringPort::Firing, priority)],
            PriorityRule::ByOutput { off, on } => {
                vec![(FiringPort::FiringOff, off), (FiringPort::FiringOn, on)]
            }
            PriorityRule::ByPulsing => vec![(FiringPort::FiringPulse, PULSE_END_PRIORITY)],
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Instance {
    pub module: NodeModule,
    pub parameters: Vec<(Parameter, u32)>,
    pub circuit_signals: Vec<CircuitSignal>,
    pub firing_ports: Vec<(FiringPort, TickPriority)>,
    pub inputs: Vec<(LinkKind, SignalKind)>,
    pub has_output: bool,
    pub power: Register,
    pub registers: Vec<Register>,
}

impl Instance {
    pub fn all_registers(&self) -> impl Iterator<Item = Register> + '_ {
        iter::once(self.power).chain(self.registers.iter().copied())
    }

    fn new(module: NodeModule, power: Register) -> Instance {
        Instance {
            module,
            parameters: Vec::new(),
            circuit_signals: Vec::new(),
            firing_ports: Vec::new(),
            inputs: Vec::new(),
            has_output: false,
            power,
            registers: Vec::new(),
        }
    }

    fn with_parameters(mut self, parameters: &[(Parameter, u32)]) -> Instance {
        self.parameters.extend(parameters);
        self
    }

    fn with_register(mut self, register: Register) -> Instance {
        self.registers.push(register);
        self
    }

    fn with_circuit_signals(mut self, circuit_signals: &[CircuitSignal]) -> Instance {
        self.circuit_signals.extend(circuit_signals);
        self
    }

    fn with_drive(self, id: NodeId, circuit_signals: &[CircuitSignal]) -> Instance {
        self.with_parameters(&[(Parameter::Index, id.index() as u32)])
            .with_circuit_signals(circuit_signals)
    }

    fn with_firing_ports(mut self, firing_ports: &[(FiringPort, TickPriority)]) -> Instance {
        self.firing_ports.extend(firing_ports);
        self
    }

    fn with_timer(self, delay: u32, priority: PriorityRule) -> Instance {
        self.with_firing_ports(&[(FiringPort::TickStart, TICK_START_PRIORITY)])
            .with_firing_ports(&priority.firing_ports())
            .with_register(Register::Countdown { delay, priority })
    }
}

pub fn lower(circuit: &Circuit) -> Result<Vec<Option<Instance>>, CompileError> {
    if circuit.len() > 1 << DRIVE_NODE_BITS {
        let count = circuit.len();
        return Err(CompileError::Unsupported(format!(
            "{count} nodes are more than the 2^{DRIVE_NODE_BITS} that fpga's protocol addresses"
        )));
    }
    let instances: Vec<_> = circuit
        .ids()
        .map(|id| instance(circuit.node(id), id))
        .collect::<Result<_, _>>()?;
    if instances.iter().all(Option::is_none) {
        return Err(CompileError::Unsupported(
            "the circuit has no component that can change, so fpga has nothing to run".to_owned(),
        ));
    }
    Ok(instances)
}

fn instance(node: &Node, id: NodeId) -> Result<Option<Instance>, CompileError> {
    use PriorityRule::{ByOutput, ByPulsing, Fixed};
    use Register::{Locked, Played, Powered, Pulsing, Strength, Touched};
    let refused = |problem: String| {
        let index = id.index();
        CompileError::Unsupported(format!("node {index} {problem}"))
    };
    let lever = || Instance::new(NodeModule::Lever, Powered).with_drive(id, CircuitSignal::DRIVE);
    let instance = match node.kind {
        NodeKind::Constant => return Ok(None),
        NodeKind::Lever => lever(),
        NodeKind::Repeater { .. } | NodeKind::Torch if node.is_input() => lever(),
        NodeKind::Comparator { .. } if node.is_input() => {
            Instance::new(NodeModule::StrengthInput, Strength).with_drive(id, CircuitSignal::DRIVE)
        }
        NodeKind::Button { press_ticks } => Instance::new(NodeModule::Button, Powered)
            .with_drive(id, CircuitSignal::PRESS)
            .with_timer(press_ticks.into(), Fixed(attached::BUTTON_PRIORITY)),
        NodeKind::PressurePlate => Instance::new(NodeModule::Plate, Powered)
            .with_drive(id, CircuitSignal::DRIVE)
            .with_register(Touched)
            .with_timer(plate::PRESS_TICKS, Fixed(plate::RECHECK_PRIORITY)),
        NodeKind::Torch => Instance::new(NodeModule::Torch, Powered)
            .with_timer(torch::DELAY_TICKS, Fixed(torch::PRIORITY)),
        NodeKind::Repeater {
            delay,
            prioritized: false,
        } => {
            let [off, on] = [false, true].map(|powered| repeater::tick_priority(false, powered));
            let repeater = Instance::new(NodeModule::Repeater, Powered).with_register(Locked);
            repeater.with_timer(delay.into(), ByOutput { off, on })
        }
        NodeKind::Repeater {
            delay,
            prioritized: true,
        } => Instance::new(NodeModule::RepeaterPrioritized, Powered)
            .with_register(Locked)
            .with_timer(delay.into(), ByPulsing)
            .with_register(Pulsing),
        NodeKind::Comparator {
            mode,
            far_input,
            prioritized,
        } => {
            let far = far_input.map_or(0, |far| far.get().into());
            let rule = [
                (
                    Parameter::Subtract,
                    (mode == ComparatorMode::Subtract).into(),
                ),
                (Parameter::FarValid, far_input.is_some().into()),
                (Parameter::Far, far),
            ];
            let priority = Fixed(comparator::tick_priority(prioritized));
            Instance::new(NodeModule::Comparator, Strength)
                .with_parameters(&rule)
                .with_timer(comparator::DELAY_TICKS, priority)
        }
        NodeKind::Lamp => Instance::new(NodeModule::Lamp, Powered)
            .with_timer(LAMP_OFF_DELAY_TICKS, Fixed(TickPriority::Normal)),
        NodeKind::Trapdoor => Instance::new(NodeModule::Trapdoor, Powered),
        NodeKind::Wire => Instance::new(NodeModule::RedstoneWire, Strength),
        NodeKind::NoteBlock { .. } => Instance::new(NodeModule::NoteBlock, Powered)
            .with_register(Played)
            .with_circuit_signals(&[CircuitSignal::ShownRead]),
    };
    let pending = node.pending_tick.map(|queued| queued.tick);
    for register in instance.all_registers() {
        if let (Register::Countdown { priority, .. }, Some(tick)) = (register, pending) {
            let held = priority.of(node.state.is_powered(), tick.priority == PULSE_END_PRIORITY);
            if held != tick.priority {
                return Err(refused(format!(
                    "has a pending tick of priority {}, which only an edit produces: fpga would \
                     run it with priority {held}",
                    tick.priority
                )));
            }
        }
    }
    let traits = node.kind.traits();
    let initial = instance
        .all_registers()
        .flat_map(|register| register.parameters(node));
    let parameters = instance.parameters.iter().copied().chain(initial);
    let read = traits.reads.filter(|_| !node.is_input());
    let side = traits.side_inputs.then_some(LinkKind::Side);
    let inputs = read.iter().flat_map(|&signal| {
        let kinds = [Some(LinkKind::Default), side].into_iter().flatten();
        kinds.map(move |kind| (kind, signal))
    });
    Ok(Some(Instance {
        parameters: parameters.collect(),
        inputs: inputs.collect(),
        has_output: traits.emits.is_some(),
        ..instance
    }))
}
