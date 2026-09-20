use super::node::{ForwardLink, ForwardLinks, Node, NodeInput, NodeType, Nodes, NonMaxU8};
use super::{DirectEngine, TickScheduler};
use crate::compile_graph::LinkType;
use crate::engine::PendingTick;
use crate::netlist::{Netlist, NodeId};
use itertools::Itertools;
use tracing::trace;

#[derive(Debug, Default)]
struct FinalGraphStats {
    update_link_count: usize,
    side_link_count: usize,
    default_link_count: usize,
    nodes_bytes: usize,
}

fn compile_node(
    netlist: &Netlist,
    id: NodeId,
    io_only: bool,
    forward_links: &mut ForwardLinks,
    stats: &mut FinalGraphStats,
) -> Node {
    let node = netlist.node(id);

    const MAX_INPUTS: usize = 255;

    let mut default_input_count = 0;
    let mut side_input_count = 0;

    let mut default_inputs = NodeInput { ss_counts: [0; 16] };
    let mut side_inputs = NodeInput { ss_counts: [0; 16] };
    for link in netlist.inputs(id) {
        let ss = netlist
            .node(link.source)
            .state
            .output_strength
            .saturating_sub(link.ss);
        match link.ty {
            LinkType::Default => {
                if default_input_count >= MAX_INPUTS {
                    panic!(
                        "Exceeded the maximum number of default inputs {}",
                        MAX_INPUTS
                    );
                }
                default_input_count += 1;
                default_inputs.ss_counts[ss as usize] += 1;
            }
            LinkType::Side => {
                if side_input_count >= MAX_INPUTS {
                    panic!("Exceeded the maximum number of side inputs {}", MAX_INPUTS);
                }
                side_input_count += 1;
                side_inputs.ss_counts[ss as usize] += 1;
            }
        }
    }
    stats.default_link_count += default_input_count;
    stats.side_link_count += side_input_count;

    // Make sure signal strength buckets add up to 255 so we can easily check for all zeros in
    // get_bool_input
    default_inputs.ss_counts[0] += (MAX_INPUTS - default_input_count) as u8;
    side_inputs.ss_counts[0] += (MAX_INPUTS - side_input_count) as u8;

    use crate::compile_graph::NodeType as CNodeType;
    let fwd_link_range = if node.ty != CNodeType::Constant {
        let new_links = netlist
            .outputs(id)
            .iter()
            .into_group_map_by(|link| std::mem::discriminant(&netlist.node(link.target).ty))
            .into_values()
            .flatten()
            .map(|link| ForwardLink::new(link.target, link.ty == LinkType::Side, link.ss));
        forward_links.extend(new_links)
    } else {
        Default::default()
    };
    stats.update_link_count += fwd_link_range.len();

    let ty = match &node.ty {
        CNodeType::Repeater {
            delay,
            facing_diode,
        } => NodeType::Repeater {
            delay: *delay,
            facing_diode: *facing_diode,
        },
        CNodeType::Torch => NodeType::Torch,
        CNodeType::Comparator {
            mode,
            far_input,
            facing_diode,
        } => NodeType::Comparator {
            mode: *mode,
            far_input: far_input.map(|value| NonMaxU8::new(value).unwrap()),
            facing_diode: *facing_diode,
        },
        CNodeType::Lamp => NodeType::Lamp,
        CNodeType::Button => NodeType::Button,
        CNodeType::Lever => NodeType::Lever,
        CNodeType::PressurePlate => NodeType::PressurePlate,
        CNodeType::Trapdoor => NodeType::Trapdoor,
        CNodeType::Wire => NodeType::Wire,
        CNodeType::Constant => NodeType::Constant,
        CNodeType::NoteBlock { .. } => NodeType::NoteBlock,
    };

    Node {
        ty,
        default_inputs,
        side_inputs,
        fwd_link_range,
        powered: node.state.powered,
        output_power: node.state.output_strength,
        locked: node.state.repeater_locked,
        changed: false,
        pending_tick: false,
        visible: !io_only || node.is_input || node.is_output,
    }
}

pub fn compile(netlist: &Netlist, io_only: bool, pending_ticks: Vec<PendingTick>) -> DirectEngine {
    let mut stats = FinalGraphStats::default();
    let mut forward_links = ForwardLinks::default();
    let nodes: Box<[Node]> = netlist
        .ids()
        .map(|id| compile_node(netlist, id, io_only, &mut forward_links, &mut stats))
        .collect();
    stats.nodes_bytes = nodes.len() * std::mem::size_of::<Node>();
    trace!("{:#?}", stats);

    let mut nodes = Nodes::new(nodes);
    let mut scheduler = TickScheduler::default();
    for tick in pending_ticks {
        nodes.inner_mut()[tick.node.index()].pending_tick = true;
        scheduler.schedule_tick(tick.node, tick.delay as usize, tick.priority);
    }

    DirectEngine {
        nodes,
        forward_links,
        scheduler,
        notes: Vec::new(),
        work: 0,
        changed_since_take: false,
    }
}
