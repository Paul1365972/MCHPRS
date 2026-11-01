use super::{
    Circuit, CircuitError, ComparatorMode, Kind, Link, Node, NodeError, PendingTick, State,
};
use mchprs_blocks::blocks::ComparatorMode as BlockComparatorMode;
use mchprs_redpiler::compile_graph::{LinkType, NodeType};
use mchprs_redpiler::{Netlist, NodeId};
use std::num::NonZeroU8;

impl Circuit {
    pub(crate) fn lower(
        netlist: &Netlist,
        pending_ticks: &[mchprs_redpiler::PendingTick],
    ) -> Result<Circuit, CircuitError> {
        let mut nodes = netlist
            .ids()
            .map(|id| lower_node(netlist, id).map_err(|problem| problem.at(id.index())))
            .collect::<Result<Vec<_>, _>>()?;
        for tick in pending_ticks {
            let index = tick.node.index();
            schedule(&mut nodes[index], tick).map_err(|problem| problem.at(index))?;
        }
        for id in netlist.ids() {
            if netlist.node(id).state.pending_tick && nodes[id.index()].initial.pending.is_none() {
                return Err(NodeError::MarkedPending.at(id.index()));
            }
        }
        Circuit::new(nodes)
    }
}

fn lower_node(netlist: &Netlist, id: NodeId) -> Result<Node, NodeError> {
    let node = netlist.node(id);
    let kind = kind(&node.ty).ok_or_else(|| NodeError::Unsupported(node.ty.clone()))?;
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
}

fn schedule(node: &mut Node, tick: &mchprs_redpiler::PendingTick) -> Result<(), NodeError> {
    if !node.kind.is_timed() {
        return Err(NodeError::Untimed(node.kind));
    }
    let delay = u8::try_from(tick.delay.max(1))
        .ok()
        .and_then(NonZeroU8::new)
        .filter(|delay| delay.get() <= 15)
        .ok_or(NodeError::PendingDelay(tick.delay))?;
    let pending = PendingTick {
        delay,
        priority: tick.priority,
    };
    if node
        .initial
        .pending
        .is_some_and(|existing| existing != pending)
    {
        return Err(NodeError::DistinctPendingTicks);
    }
    node.initial.pending = Some(pending);
    Ok(())
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
