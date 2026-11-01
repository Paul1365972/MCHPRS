use super::node::NodeId;
use super::*;

impl DirectEngine {
    // Benchmarks show that `tick_node` getting inlined into `tick` causes worse perf.
    #[inline(never)]
    pub fn tick_node(&mut self, node_id: NodeId) {
        let node = &mut self.nodes[node_id];
        // Lamps clear their pending tick when it is collected, see `DirectEngine::tick`
        if !matches!(node.kind, NodeKind::Lamp) {
            node.pending_tick = false;
        }

        match node.kind {
            NodeKind::Repeater { delay, .. } => {
                if node.repeater_locked {
                    return;
                }

                let should_be_powered = node.default_inputs.is_powered();
                if node.is_powered() && !should_be_powered {
                    self.set_power_and_propagate(node_id, SignalStrength::Zero);
                } else if !node.is_powered() {
                    if !should_be_powered {
                        schedule_tick(
                            &mut self.scheduler,
                            node_id,
                            node,
                            delay as usize,
                            TickPriority::Higher,
                        );
                    }
                    self.set_power_and_propagate(node_id, SignalStrength::Fifteen);
                }
            }
            NodeKind::Torch => {
                let should_be_powered = !node.default_inputs.is_powered();
                if node.is_powered() != should_be_powered {
                    self.set_power_and_propagate(node_id, should_be_powered.into());
                }
            }
            NodeKind::Comparator {
                mode, far_input, ..
            } => {
                let power = comparator_output_power(node, mode, far_input);
                if power != node.power {
                    self.set_power_and_propagate(node_id, power);
                }
            }
            NodeKind::Lamp => {
                let should_be_lit = node.default_inputs.is_powered();
                if node.is_powered() && !should_be_lit {
                    node.set_powered(false);
                }
            }
            NodeKind::Button => {
                if node.is_powered() {
                    self.set_power_and_propagate(node_id, SignalStrength::Zero);
                }
            }
            _ => unreachable!(),
        }
    }
}
