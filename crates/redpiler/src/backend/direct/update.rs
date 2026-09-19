use mchprs_world::TickPriority;

use super::node::{Gate, NodeId, NodeType};
use super::{DirectBackend, Event};

impl DirectBackend {
    #[inline(always)]
    pub(super) fn update_node(&mut self, node_id: NodeId) {
        match self.nodes[node_id].ty {
            NodeType::Gate(gate) => self.update_gate(node_id, gate),
            _ => self.update_other(node_id),
        }
    }

    #[inline(always)]
    fn update_gate(&mut self, node_id: NodeId, gate: Gate) {
        let node = &self.nodes[node_id];
        let should_be_locked = node.side_inputs.is_powered();
        if should_be_locked != node.locked {
            self.nodes.set_locked(node_id, should_be_locked);
        }

        let node = &self.nodes[node_id];
        if node.locked || node.pending_tick {
            return;
        }
        let should_be_powered = gate.should_be_powered(&node.default_inputs);
        if should_be_powered != node.is_powered() {
            let priority = gate.tick_priority(should_be_powered);
            self.schedule_tick(node_id, gate.delay as usize, priority);
        }
    }

    // Kept out of line so the gate path carries no jump table.
    #[inline(never)]
    fn update_other(&mut self, node_id: NodeId) {
        let node = &self.nodes[node_id];
        match node.ty {
            NodeType::Gate(_) => unreachable!("gates update through update_gate"),
            NodeType::Comparator(comparator) => {
                if node.pending_tick {
                    return;
                }
                let power = comparator.output_power(&node.default_inputs, &node.side_inputs);
                if power != node.power {
                    self.schedule_tick(node_id, 1, comparator.tick_priority);
                }
            }
            NodeType::Lamp => {
                let should_be_lit = node.default_inputs.is_powered();
                let lit = node.is_powered();
                if lit && !should_be_lit {
                    if !node.pending_tick {
                        self.schedule_tick(node_id, 2, TickPriority::Normal);
                    }
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
            NodeType::Button | NodeType::Lever | NodeType::PressurePlate | NodeType::Constant => {}
        }
    }
}
