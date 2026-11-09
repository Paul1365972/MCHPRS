use crate::node::{ForwardLink, ForwardLinkRange, ForwardLinks, Node, NodeInput, Nodes};
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::graph::LinkKind;
use mchprs_simulation::backend::CompileError;

const MAX_LINKS_PER_KIND: usize = u8::MAX as usize;

pub fn lower(circuit: &Circuit) -> Result<(Nodes, ForwardLinks), CompileError> {
    if circuit.len() > ForwardLink::MAX_NODES {
        return Err(CompileError::Unsupported(format!(
            "the circuit has {} nodes (at most {} supported)",
            circuit.len(),
            ForwardLink::MAX_NODES
        )));
    }
    let nodes = circuit
        .ids()
        .map(|id| lower_node(circuit, id))
        .collect::<Result<_, _>>()?;
    let mut nodes = Nodes::new(nodes);
    let mut forward_links = ForwardLinks::default();
    for id in circuit.ids() {
        let range = forward_links.extend(circuit.outputs(id).iter().map(|link| {
            let target = nodes.get(link.target.index());
            ForwardLink::new(target, link.kind == LinkKind::Side, link.weight)
        }));
        let slot = nodes.get(id.index());
        nodes[slot].forward_links = range;
    }
    Ok((nodes, forward_links))
}

fn lower_node(circuit: &Circuit, id: NodeId) -> Result<Node, CompileError> {
    let node = circuit.node(id);
    for (link_kind, inputs) in [
        (LinkKind::Default, "inputs"),
        (LinkKind::Side, "side inputs"),
    ] {
        let links = circuit.inputs(id).iter();
        let count = links.filter(|link| link.kind == link_kind).count();
        if count > MAX_LINKS_PER_KIND {
            let component = match node.blocks.first() {
                Some((pos, _)) => format!("the component at {pos}"),
                None => format!("node {}", id.index()),
            };
            return Err(CompileError::Unsupported(format!(
                "{component} has {count} {inputs} (at most {MAX_LINKS_PER_KIND} supported)"
            )));
        }
    }
    Ok(Node {
        kind: node.kind,
        default_inputs: NodeInput::new(),
        side_inputs: NodeInput::new(),
        forward_links: ForwardLinkRange::default(),
        state: node.state,
        changed: false,
        played_note: false,
        due_tick: None,
    })
}
