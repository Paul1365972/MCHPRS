use tracing::warn;

use crate::redpiler::backend::common::NodeIdWithData;

use super::node::NodeType;
use super::*;

impl Worker {
    pub fn tick_node(&mut self, node_data: NodeIdWithData) {
        let node_id = node_data.node();
        let internal_tick = node_data.internal_tick();
        let node = &mut self.nodes[node_id];
        if !internal_tick {
            node.pending_tick = false;
        }

        match node.ty {
            NodeType::Repeater { delay, .. } => {
                if node.side_power > 0 {
                    return;
                }

                let should_be_powered = get_bool_input(node);
                if node.powered && !should_be_powered {
                    self.set_node(node_id, false, 0);
                } else if !node.powered {
                    if !should_be_powered {
                        schedule_tick(
                            &mut self.scheduler,
                            NodeIdWithData::new(node_id),
                            node,
                            delay as usize,
                            TickPriority::Higher,
                        );
                    }
                    self.set_node(node_id, true, 15);
                }
            }
            NodeType::Torch => {
                let should_be_off = get_bool_input(node);
                let lit = node.powered;
                if lit && should_be_off {
                    self.set_node(node_id, false, 0);
                } else if !lit && !should_be_off {
                    self.set_node(node_id, true, 15);
                }
            }
            NodeType::Comparator {
                mode, far_input, ..
            } => {
                let mut input_power = get_analog_input(node);
                let side_input_power = get_analog_side(node);
                if let Some(far_override) = far_input {
                    if input_power < 15 {
                        input_power = far_override.get();
                    }
                }
                let old_strength = node.output_power;
                let new_strength = calculate_comparator_output(mode, input_power, side_input_power);
                if new_strength != old_strength {
                    self.set_node(node_id, new_strength > 0, new_strength);
                }
            }
            NodeType::Lamp => {
                let should_be_lit = get_bool_input(node);
                if node.powered && !should_be_lit {
                    self.set_node(node_id, false, 0);
                }
            }
            NodeType::Button => {
                if node.powered {
                    self.set_node(node_id, false, 0);
                }
            }
            NodeType::ComparatorLine { delay } => {
                if internal_tick {
                    let new_strength = node_data.ss();
                    if new_strength != node.output_power {
                        self.set_node(node_id, new_strength > 0, new_strength);
                    }
                } else {
                    let input_power = get_analog_input(node);
                    schedule_tick(
                        &mut self.scheduler,
                        NodeIdWithData::new_with_data(node_id, true, input_power),
                        node,
                        delay as usize - 1,
                        TickPriority::Normal,
                    );
                    node.pending_tick = false;
                }
            }
            _ => warn!("Node {:?} should not be ticked!", node.ty),
        }
    }
}
