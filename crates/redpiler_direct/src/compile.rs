use crate::node::{ForwardLink, ForwardLinks, Node, NodeId, NodeInput, NodeKind};
use mchprs_redpiler::circuit::{self, Circuit};
use mchprs_redpiler::engine::EngineError;
use mchprs_redpiler::graph::{self, LinkKind};

const MAX_LINKS_PER_KIND: usize = u8::MAX as usize;

pub fn lower(circuit: &Circuit) -> Result<(Box<[Node]>, ForwardLinks), EngineError> {
    for id in circuit.ids() {
        for kind in [LinkKind::Default, LinkKind::Side] {
            let count = circuit
                .inputs(id)
                .iter()
                .filter(|link| link.kind == kind)
                .count();
            if count > MAX_LINKS_PER_KIND {
                return Err(format!(
                    "node {} has {count} {kind:?} inputs (at most {} supported)",
                    id.index(),
                    MAX_LINKS_PER_KIND
                )
                .into());
            }
        }
    }
    let mut forward_links = ForwardLinks::default();
    let nodes = circuit
        .ids()
        .map(|id| build_node(circuit, id, &mut forward_links))
        .collect();
    Ok((nodes, forward_links))
}

fn build_node(circuit: &Circuit, id: circuit::NodeId, forward_links: &mut ForwardLinks) -> Node {
    let node = circuit.node(id);
    let fwd_link_range = forward_links.extend(circuit.outputs(id).iter().map(|link| {
        let target = NodeId::new(link.target.index(), circuit.len());
        ForwardLink::new(target, link.kind == LinkKind::Side, link.weight)
    }));

    let kind = match node.kind {
        graph::NodeKind::Repeater { delay, prioritized } => {
            NodeKind::Repeater { delay, prioritized }
        }
        graph::NodeKind::Torch => NodeKind::Torch,
        graph::NodeKind::Comparator {
            mode,
            far_input,
            prioritized,
        } => NodeKind::Comparator {
            mode,
            far_input,
            prioritized,
        },
        graph::NodeKind::Lamp => NodeKind::Lamp,
        graph::NodeKind::Button => NodeKind::Button,
        graph::NodeKind::Lever => NodeKind::Lever,
        graph::NodeKind::PressurePlate => NodeKind::PressurePlate,
        graph::NodeKind::Trapdoor => NodeKind::Trapdoor,
        graph::NodeKind::Wire => NodeKind::Wire,
        graph::NodeKind::Constant => NodeKind::Constant,
        graph::NodeKind::NoteBlock { .. } => NodeKind::NoteBlock,
    };

    Node {
        kind,
        default_inputs: NodeInput::new(),
        side_inputs: NodeInput::new(),
        fwd_link_range,
        power: node.state.power,
        repeater_locked: node.state.repeater_locked,
        changed: false,
        pending_tick: false,
        visible: node.observed,
    }
}
