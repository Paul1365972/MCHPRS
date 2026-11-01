use mchprs_redstone::tick_queue::TickPriority;

use super::node::{NodeId, NodeKind};
use super::*;

#[inline(always)]
pub(super) fn update_node(
    scheduler: &mut TickScheduler,
    noteblock_info: &mut [NoteBlockInfo],
    nodes: &mut Nodes,
    node_id: NodeId,
) {
    let node = &mut nodes[node_id];

    match node.kind {
        NodeKind::Repeater { delay, prioritized } => {
            let should_be_locked = node.side_inputs.is_powered();
            if should_be_locked != node.repeater_locked {
                node.set_repeater_locked(should_be_locked);
            }
            if node.repeater_locked || node.pending_tick {
                return;
            }

            let should_be_powered = node.default_inputs.is_powered();
            if should_be_powered != node.is_powered() {
                let priority = if prioritized {
                    TickPriority::Highest
                } else if !should_be_powered {
                    TickPriority::Higher
                } else {
                    TickPriority::High
                };
                schedule_tick(scheduler, node_id, node, delay as usize, priority);
            }
        }
        NodeKind::Torch => {
            if node.pending_tick {
                return;
            }
            let should_be_powered = !node.default_inputs.is_powered();
            if node.is_powered() != should_be_powered {
                schedule_tick(scheduler, node_id, node, 1, TickPriority::Normal);
            }
        }
        NodeKind::Comparator {
            mode,
            far_input,
            prioritized,
        } => {
            if node.pending_tick {
                return;
            }
            let power = comparator_output_power(node, mode, far_input);
            if power != node.power {
                let priority = if prioritized {
                    TickPriority::High
                } else {
                    TickPriority::Normal
                };
                schedule_tick(scheduler, node_id, node, 1, priority);
            }
        }
        NodeKind::Lamp => {
            let should_be_lit = node.default_inputs.is_powered();
            let lit = node.is_powered();
            if lit && !should_be_lit && !node.pending_tick {
                schedule_tick(scheduler, node_id, node, 2, TickPriority::Normal);
            } else if !lit && should_be_lit {
                node.set_powered(true);
            }
        }
        NodeKind::Trapdoor => {
            let should_be_powered = node.default_inputs.is_powered();
            if node.is_powered() != should_be_powered {
                node.set_powered(should_be_powered);
            }
        }
        NodeKind::Wire => {
            let input_power = node.default_inputs.power();
            if node.power != input_power {
                node.set_power(input_power);
            }
        }
        NodeKind::NoteBlock { noteblock_id } => {
            let should_be_powered = node.default_inputs.is_powered();
            if node.is_powered() != should_be_powered {
                node.set_powered(should_be_powered);
                if should_be_powered {
                    noteblock_info[noteblock_id as usize].pending = true;
                }
            }
        }
        _ => {}
    }
}
