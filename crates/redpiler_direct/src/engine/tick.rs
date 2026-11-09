use super::{schedule_tick, DirectEngine};
use crate::node::Slot;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::behavior::comparator_output;
use mchprs_redpiler::graph::NodeKind;
use mchprs_redstone::tick_queue::TickPriority;

impl DirectEngine {
    // Benchmarks show that `tick_node` getting inlined into `tick` causes worse perf.
    #[inline(never)]
    pub fn tick_node(&mut self, slot: Slot) {
        let node = &mut self.nodes[slot];
        if node.due_tick == Some(self.scheduler.current_tick()) {
            node.due_tick = None;
        }

        match node.kind {
            NodeKind::Repeater { delay, .. } => {
                if node.state.repeater_locked {
                    return;
                }

                let should_be_powered = node.default_inputs.is_powered();
                if node.is_powered() && !should_be_powered {
                    self.set_power_and_propagate(slot, SignalStrength::Zero);
                } else if !node.is_powered() {
                    if !should_be_powered {
                        schedule_tick(
                            &mut self.scheduler,
                            slot,
                            node,
                            delay.into(),
                            TickPriority::Higher,
                        );
                    }
                    self.set_power_and_propagate(slot, SignalStrength::Fifteen);
                }
            }
            NodeKind::Torch => {
                let should_be_powered = !node.default_inputs.is_powered();
                if node.is_powered() != should_be_powered {
                    self.set_power_and_propagate(slot, should_be_powered.into());
                }
            }
            NodeKind::Comparator {
                mode, far_input, ..
            } => {
                let power = comparator_output(mode, far_input, node.inputs());
                if power != node.state.power {
                    self.set_power_and_propagate(slot, power);
                }
            }
            kind => {
                let outcome = kind.tick(node.state, node.inputs());
                self.apply(slot, outcome);
            }
        }
    }
}
