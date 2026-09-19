use mchprs_world::TickPriority;

use super::node::{NodeId, NodeType};
use super::{comparator_output_power, DirectBackend, Event};

impl DirectBackend {
    #[inline(always)]
    pub(super) fn update_node(&mut self, node_id: NodeId) {
        let node = &self.nodes[node_id];

        match node.ty {
            NodeType::Repeater {
                delay,
                facing_diode,
            } => {
                let should_be_locked = node.side_inputs.is_powered();
                if should_be_locked != node.repeater_locked {
                    self.nodes.set_repeater_locked(node_id, should_be_locked);
                }
                let node = &self.nodes[node_id];
                if node.repeater_locked || node.pending_tick {
                    return;
                }

                let should_be_powered = node.default_inputs.is_powered();
                if should_be_powered != node.is_powered() {
                    let priority = if facing_diode {
                        TickPriority::Highest
                    } else if !should_be_powered {
                        TickPriority::Higher
                    } else {
                        TickPriority::High
                    };
                    self.schedule_tick(node_id, delay as usize, priority);
                }
            }
            NodeType::Torch => {
                if node.pending_tick {
                    return;
                }
                let should_be_powered = !node.default_inputs.is_powered();
                if node.is_powered() != should_be_powered {
                    self.schedule_tick(node_id, 1, TickPriority::Normal);
                }
            }
            NodeType::Comparator {
                mode,
                far_input,
                facing_diode,
            } => {
                if node.pending_tick {
                    return;
                }
                let power = comparator_output_power(node, mode, far_input);
                if power != node.power {
                    let priority = if facing_diode {
                        TickPriority::High
                    } else {
                        TickPriority::Normal
                    };
                    self.schedule_tick(node_id, 1, priority);
                }
            }
            NodeType::Lamp => {
                let should_be_lit = node.default_inputs.is_powered();
                let lit = node.is_powered();
                if lit && !should_be_lit {
                    self.schedule_tick(node_id, 2, TickPriority::Normal);
                } else if !lit && should_be_lit {
                    self.nodes.set_powered(node_id, true);
                }
            }
            NodeType::Trapdoor => {
                let should_be_powered = node.default_inputs.is_powered();
                if node.is_powered() != should_be_powered {
                    self.nodes.set_powered(node_id, should_be_powered);
                }
            }
            NodeType::Wire => {
                let input_power = node.default_inputs.power();
                if node.power != input_power {
                    self.nodes.set_power(node_id, input_power);
                }
            }
            NodeType::NoteBlock { noteblock_id } => {
                let should_be_powered = node.default_inputs.is_powered();
                if node.is_powered() != should_be_powered {
                    self.nodes.set_powered(node_id, should_be_powered);
                    if should_be_powered {
                        self.events.push(Event::NoteBlockPlay { noteblock_id });
                    }
                }
            }
            _ => {}
        }
    }
}
