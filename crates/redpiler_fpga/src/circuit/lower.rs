use super::{invalid, Circuit, ComparatorMode, Kind, Link, Node, PendingTick, State};
use mchprs_blocks::blocks::ComparatorMode as BlockComparatorMode;
use mchprs_redpiler::compile_graph::{LinkType, NodeType};
use mchprs_redpiler::Netlist;
use std::io;
use std::num::NonZeroU8;

impl Circuit {
    pub(crate) fn lower(
        netlist: &Netlist,
        pending_ticks: &[mchprs_redpiler::PendingTick],
    ) -> io::Result<Circuit> {
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
                        strength: node.state.power,
                        locked: node.state.repeater_locked,
                        pending: None,
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
            let delay = u8::try_from(tick.delay.max(1))
                .ok()
                .and_then(NonZeroU8::new)
                .filter(|delay| delay.get() <= 15)
                .ok_or_else(|| {
                    invalid(format!(
                        "node {index}: pending delay {} exceeds 15",
                        tick.delay
                    ))
                })?;
            let pending = PendingTick {
                delay,
                priority: tick.priority,
            };
            if node
                .initial
                .pending
                .is_some_and(|existing| existing != pending)
            {
                return Err(invalid(format!(
                    "node {index}: distinct pending ticks cannot be represented"
                )));
            }
            node.initial.pending = Some(pending);
        }
        for id in netlist.ids() {
            if netlist.node(id).state.pending_tick && nodes[id.index()].initial.pending.is_none() {
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
            far_input,
            facing_diode,
        },
        NodeType::Lamp => Kind::Lamp,
        NodeType::Trapdoor => Kind::Trapdoor,
        NodeType::Wire => Kind::Wire,
        NodeType::Constant => Kind::Constant,
        NodeType::NoteBlock { .. } => return None,
    })
}
