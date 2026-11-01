use super::node::{ForwardLink, ForwardLinks, Node, NodeInput, NodeType, Nodes};
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

    let input_powers = |ty| {
        netlist
            .inputs(id)
            .iter()
            .filter(move |link| link.ty == ty)
            .map(|link| {
                netlist
                    .node(link.source)
                    .state
                    .power
                    .saturating_sub(link.weight)
            })
    };
    let default_inputs = input_powers(LinkType::Default)
        .inspect(|_| stats.default_link_count += 1)
        .collect::<NodeInput>();
    let side_inputs = input_powers(LinkType::Side)
        .inspect(|_| stats.side_link_count += 1)
        .collect::<NodeInput>();

    use crate::compile_graph::NodeType as CompileNodeType;
    let fwd_link_range = if node.ty != CompileNodeType::Constant {
        let new_links = netlist
            .outputs(id)
            .iter()
            .into_group_map_by(|link| std::mem::discriminant(&netlist.node(link.target).ty))
            .into_values()
            .flatten()
            .map(|link| ForwardLink::new(link.target, link.ty == LinkType::Side, link.weight));
        forward_links.extend(new_links)
    } else {
        Default::default()
    };
    stats.update_link_count += fwd_link_range.len();

    let ty = match &node.ty {
        CompileNodeType::Repeater {
            delay,
            facing_diode,
        } => NodeType::Repeater {
            delay: *delay,
            facing_diode: *facing_diode,
        },
        CompileNodeType::Torch => NodeType::Torch,
        CompileNodeType::Comparator {
            mode,
            far_input,
            facing_diode,
        } => NodeType::Comparator {
            mode: *mode,
            far_input: *far_input,
            facing_diode: *facing_diode,
        },
        CompileNodeType::Lamp => NodeType::Lamp,
        CompileNodeType::Button => NodeType::Button,
        CompileNodeType::Lever => NodeType::Lever,
        CompileNodeType::PressurePlate => NodeType::PressurePlate,
        CompileNodeType::Trapdoor => NodeType::Trapdoor,
        CompileNodeType::Wire => NodeType::Wire,
        CompileNodeType::Constant => NodeType::Constant,
        CompileNodeType::NoteBlock { .. } => NodeType::NoteBlock,
    };

    Node {
        ty,
        default_inputs,
        side_inputs,
        fwd_link_range,
        power: node.state.power,
        repeater_locked: node.state.repeater_locked,
        changed: false,
        pending_tick: false,
        visible: !io_only || node.is_input || node.is_output,
    }
}

impl DirectEngine {
    pub fn new(netlist: &Netlist, io_only: bool, pending_ticks: &[PendingTick]) -> DirectEngine {
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
            changed_since_take: false,
        }
    }
}
