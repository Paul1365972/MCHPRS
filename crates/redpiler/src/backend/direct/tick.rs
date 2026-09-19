use mchprs_world::TickPriority;

use super::node::{Gate, GateKind, NodeId, NodeType};
use super::DirectBackend;
use crate::compile_graph::SignalStrength;

impl DirectBackend {
    // Benchmarks show that `tick_node` getting inlined into `tick` causes worse perf.
    #[inline(never)]
    pub(super) fn tick_node(&mut self, node_id: NodeId) {
        let node = &mut self.nodes[node_id];
        node.pending_tick = false;
        match node.ty {
            NodeType::Gate(gate) => self.tick_gate(node_id, gate),
            _ => self.tick_other(node_id),
        }
    }

    #[inline(always)]
    fn tick_gate(&mut self, node_id: NodeId, gate: Gate) {
        let node = &self.nodes[node_id];
        if node.locked {
            return;
        }

        let should_be_powered = gate.should_be_powered(&node.default_inputs);
        if node.is_powered() {
            if !should_be_powered {
                self.set_power_and_propagate(node_id, SignalStrength::ZERO);
            }
        } else if should_be_powered {
            self.set_power_and_propagate(node_id, SignalStrength::MAX);
        } else if gate.kind == GateKind::Repeater {
            self.schedule_tick(node_id, gate.delay as usize, TickPriority::Higher);
            self.set_power_and_propagate(node_id, SignalStrength::MAX);
        }
    }

    #[inline(never)]
    fn tick_other(&mut self, node_id: NodeId) {
        let node = &mut self.nodes[node_id];
        match node.ty {
            NodeType::Gate(_) => unreachable!("gates tick through tick_gate"),
            NodeType::Comparator(comparator) => {
                let power = comparator.output_power(&node.default_inputs, &node.side_inputs);
                if power != node.power {
                    self.set_power_and_propagate(node_id, power);
                }
            }
            NodeType::Lamp => {
                let should_be_lit = node.default_inputs.is_powered();
                if node.is_powered() && !should_be_lit {
                    self.nodes.set_powered(node_id, false);
                }
            }
            NodeType::Button => {
                if node.is_powered() {
                    self.set_power_and_propagate(node_id, SignalStrength::ZERO);
                }
            }
            NodeType::Lever
            | NodeType::PressurePlate
            | NodeType::Trapdoor
            | NodeType::Wire
            | NodeType::Constant
            | NodeType::NoteBlock { .. } => {}
        }
    }
}
