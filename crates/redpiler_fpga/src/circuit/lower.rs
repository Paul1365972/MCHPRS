use super::{invalid, Circuit, ComparatorMode, Kind, Link, Node, State};
use mchprs_blocks::blocks::ComparatorMode as BlockComparatorMode;
use mchprs_redpiler::compile_graph::{LinkType, NodeType, SignalStrength};
use mchprs_redpiler::{Netlist, PendingTick};
use mchprs_world::TickPriority;
use std::io;

impl Circuit {
    pub(crate) fn lower(netlist: &Netlist, pending_ticks: &[PendingTick]) -> io::Result<Circuit> {
        let mut nodes = netlist
            .ids()
            .map(|id| {
                let node = netlist.node(id);
                let kind = kind(&node.ty).ok_or_else(|| {
                    invalid(format!(
                        "node {}: {:?} is not supported",
                        id.index(),
                        node.ty
                    ))
                })?;
                let mut inputs = Vec::new();
                let mut sides = Vec::new();
                for link in netlist.inputs(id) {
                    let lowered = Link {
                        source: link.source.index(),
                        attenuation: link.weight,
                    };
                    match link.ty {
                        LinkType::Default => inputs.push(lowered),
                        LinkType::Side => sides.push(lowered),
                    }
                }
                Ok(Node {
                    kind,
                    initial: State {
                        strength: node.state.power.get(),
                        powered: node.state.is_powered(),
                        locked: node.state.repeater_locked,
                        open: kind == Kind::Trapdoor && node.state.is_powered(),
                        ..State::default()
                    },
                    inputs,
                    sides,
                    is_input: node.is_input,
                    is_output: node.is_output,
                })
            })
            .collect::<io::Result<Vec<_>>>()?;
        for tick in pending_ticks {
            let index = tick.node.index();
            let node = &mut nodes[index];
            if !node.kind.is_timed() {
                return Err(invalid(format!(
                    "node {index}: {:?} cannot have a pending tick",
                    node.kind
                )));
            }
            let delay = tick.delay.max(1);
            if delay > 15 {
                return Err(invalid(format!(
                    "node {index}: pending delay {delay} exceeds 15"
                )));
            }
            let delay = delay as u8;
            let priority = priority_index(tick.priority);
            let state = &mut node.initial;
            if state.delay != 0 && (state.delay, state.priority) != (delay, priority) {
                return Err(invalid(format!(
                    "node {index}: distinct pending ticks cannot be represented"
                )));
            }
            state.delay = delay;
            state.priority = priority;
        }
        for id in netlist.ids() {
            if netlist.node(id).state.pending_tick && nodes[id.index()].initial.delay == 0 {
                return Err(invalid(format!(
                    "node {}: marked pending without a pending tick",
                    id.index()
                )));
            }
        }
        Circuit::new(nodes)
    }
}

fn kind(node_type: &NodeType) -> Option<Kind> {
    Some(match *node_type {
        NodeType::Lever => Kind::Lever,
        NodeType::Button => Kind::Button,
        NodeType::PressurePlate => Kind::PressurePlate,
        NodeType::Torch => Kind::Torch,
        NodeType::Repeater {
            delay,
            facing_diode,
        } => Kind::Repeater {
            delay,
            facing_diode,
        },
        NodeType::Comparator {
            mode,
            far_input,
            facing_diode,
        } => Kind::Comparator {
            mode: match mode {
                BlockComparatorMode::Compare => ComparatorMode::Compare,
                BlockComparatorMode::Subtract => ComparatorMode::Subtract,
            },
            far_input: far_input.map(SignalStrength::get),
            facing_diode,
        },
        NodeType::Lamp => Kind::Lamp,
        NodeType::Trapdoor => Kind::Trapdoor,
        NodeType::Wire => Kind::Wire,
        NodeType::Constant => Kind::Constant,
        NodeType::NoteBlock { .. } => return None,
    })
}

const PRIORITIES: [TickPriority; 4] = [
    TickPriority::Highest,
    TickPriority::Higher,
    TickPriority::High,
    TickPriority::Normal,
];

fn priority_index(priority: TickPriority) -> u8 {
    PRIORITIES
        .iter()
        .position(|&candidate| candidate == priority)
        .unwrap() as u8
}

pub(crate) fn priority(index: u8) -> TickPriority {
    PRIORITIES[usize::from(index)]
}
