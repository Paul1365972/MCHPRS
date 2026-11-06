use crate::lower::{Instance, PriorityRule, Register};
use crate::protocol::Contents;
use anyhow::ensure;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::graph::NodeState;
use mchprs_redpiler::nodes::Snapshot;
use mchprs_redstone::tick_queue::PendingTick;

const NARROWEST_PORT_BITS: usize = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadbackScope {
    All,
    Observed,
}

/// The shown registers of all nodes come first in the state vector, so a read of the shown state
/// is a prefix of the whole.
#[derive(Debug, Clone)]
pub struct StateLayout {
    scope: ReadbackScope,
    registers: Vec<Vec<(Register, usize)>>,
    initial_states: Vec<NodeState>,
    shown_nodes: Vec<NodeId>,
    shown_bits: usize,
    state_bits: usize,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ShownState {
    pub states: Vec<(NodeId, NodeState)>,
    pub played_note_blocks: Vec<NodeId>,
}

#[derive(Default)]
struct RegisterValues {
    state: NodeState,
    played: bool,
    timer: Option<TimerValues>,
    pulsing: bool,
}

struct TimerValues {
    countdown: u32,
    delay: u32,
    priority: PriorityRule,
}

impl StateLayout {
    pub fn new(
        circuit: &Circuit,
        instances: &[Option<Instance>],
        scope: ReadbackScope,
    ) -> StateLayout {
        let is_shown_node = |node: NodeId| match scope {
            ReadbackScope::All => true,
            ReadbackScope::Observed => circuit.node(node).observed,
        };
        let is_read = |node: NodeId, register: &Register| match scope {
            ReadbackScope::All => true,
            ReadbackScope::Observed => is_shown_node(node) && register.is_shown(),
        };
        let mut registers = vec![Vec::new(); instances.len()];
        let mut state_bits = 0;
        let mut place = |shown: bool| {
            let nodes = registers.iter_mut().zip(instances).zip(circuit.ids());
            for ((placed, instance), node) in nodes {
                let all = instance.iter().flat_map(Instance::all_registers);
                let read = all.filter(|register| register.is_shown() == shown);
                for register in read.filter(|register| is_read(node, register)) {
                    placed.push((register, state_bits));
                    state_bits += register.width();
                }
            }
            state_bits
        };
        let shown_bits = place(true);
        let state_bits = place(false);
        StateLayout {
            scope,
            registers,
            initial_states: circuit.initial().states,
            shown_nodes: circuit.ids().filter(|&node| is_shown_node(node)).collect(),
            shown_bits: shown_bits.max(NARROWEST_PORT_BITS),
            state_bits: state_bits.max(NARROWEST_PORT_BITS),
        }
    }

    pub fn is_complete(&self) -> bool {
        self.scope == ReadbackScope::All
    }

    pub fn holds_no_register(&self) -> bool {
        self.registers.iter().all(Vec::is_empty)
    }

    pub fn registers(&self, node: NodeId) -> &[(Register, usize)] {
        &self.registers[node.index()]
    }

    pub fn bits(&self, contents: Contents) -> usize {
        match contents {
            Contents::Shown => self.shown_bits,
            Contents::All => self.state_bits,
        }
    }

    pub fn decode_shown(&self, state: &[u8]) -> anyhow::Result<ShownState> {
        self.check_length(Contents::Shown, state)?;
        let mut shown = ShownState::default();
        for &node in &self.shown_nodes {
            let registers = self.register_values(node, state, Contents::Shown);
            shown.states.push((node, registers.state.shown()));
            if registers.played {
                shown.played_note_blocks.push(node);
            }
        }
        Ok(shown)
    }

    fn check_length(&self, contents: Contents, state: &[u8]) -> anyhow::Result<()> {
        let expected = self.bits(contents).div_ceil(8);
        let actual = state.len();
        ensure!(
            actual == expected,
            "the board sent {actual} bytes of state where the circuit has {expected}"
        );
        Ok(())
    }

    fn register_values(&self, node: NodeId, state: &[u8], contents: Contents) -> RegisterValues {
        let mut registers = RegisterValues {
            state: self.initial_states[node.index()],
            ..RegisterValues::default()
        };
        let read = self.registers[node.index()].iter();
        let read = read.filter(|(register, _)| match contents {
            Contents::Shown => register.is_shown(),
            Contents::All => true,
        });
        for &(register, offset) in read {
            let value = read_bits(state, offset, register.width());
            let on = value != 0;
            let state = &mut registers.state;
            match register {
                Register::Powered => state.power = on.into(),
                Register::Strength => {
                    state.power =
                        SignalStrength::try_from(value).expect("four bits hold a strength");
                }
                Register::BinaryStrength { low, high } => state.power = if on { high } else { low },
                Register::Locked => state.repeater_locked = on,
                Register::Played => registers.played = on,
                Register::Touched => state.plate_touched = on,
                Register::Countdown { delay, priority } => {
                    registers.timer = Some(TimerValues {
                        countdown: value,
                        delay,
                        priority,
                    });
                }
                Register::Pulsing => registers.pulsing = on,
            }
        }
        registers
    }

    pub fn decode_snapshot(&self, state: &[u8]) -> anyhow::Result<Snapshot> {
        ensure!(
            self.is_complete(),
            "the board does not read back every register"
        );
        self.check_length(Contents::All, state)?;
        let mut snapshot = Snapshot::default();
        for node in (0..self.registers.len()).map(NodeId::new) {
            let registers = self.register_values(node, state, Contents::All);
            snapshot.states.push(registers.state);
            if let Some(TimerValues {
                countdown,
                delay,
                priority,
            }) = registers.timer
            {
                let priority = priority.of(registers.state.is_powered(), registers.pulsing);
                let pending = (1..=delay).filter(|delay| countdown >> (delay - 1) & 1 != 0);
                let pending = pending.map(|delay| (node, PendingTick { delay, priority }));
                snapshot.pending_ticks.extend(pending);
            }
        }
        Ok(snapshot)
    }
}

fn read_bits(state: &[u8], offset: usize, width: usize) -> u32 {
    let bits = offset..offset + width;
    bits.rev().fold(0, |value, bit| {
        value << 1 | u32::from(state[bit / 8] >> (bit % 8) & 1)
    })
}
