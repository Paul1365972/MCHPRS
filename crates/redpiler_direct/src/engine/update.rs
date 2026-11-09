use super::schedule_tick;
use crate::node::{Node, Nodes, Slot};
use crate::scheduler::TickScheduler;
use mchprs_redpiler::behavior::comparator_output;
use mchprs_redpiler::graph::NodeKind;
use mchprs_redstone::{comparator, repeater, torch};

#[inline(always)]
pub fn update_node(scheduler: &mut TickScheduler, nodes: &mut Nodes, slot: Slot) {
    let node = &mut nodes[slot];

    match node.kind {
        NodeKind::Repeater { delay, prioritized } => {
            let should_be_locked = node.side_inputs.is_powered();
            if should_be_locked != node.state.repeater_locked {
                node.set_repeater_locked(should_be_locked);
            }
            if node.state.repeater_locked || node.has_pending_tick() {
                return;
            }

            let should_be_powered = node.default_inputs.is_powered();
            if should_be_powered != node.is_powered() {
                let priority = repeater::tick_priority(prioritized, node.is_powered());
                schedule_tick(scheduler, slot, node, delay.into(), priority);
            }
        }
        NodeKind::Torch => {
            if node.has_pending_tick() {
                return;
            }
            let should_be_powered = !node.default_inputs.is_powered();
            if node.is_powered() != should_be_powered {
                schedule_tick(scheduler, slot, node, torch::DELAY_TICKS, torch::PRIORITY);
            }
        }
        NodeKind::Comparator {
            mode,
            far_input,
            prioritized,
        } => {
            if node.has_pending_tick() {
                return;
            }
            let power = comparator_output(mode, far_input, node.inputs());
            if power != node.state.power {
                let priority = comparator::tick_priority(prioritized);
                schedule_tick(scheduler, slot, node, comparator::DELAY_TICKS, priority);
            }
        }
        _ => update_by_spec(scheduler, node, slot),
    }
}

#[inline(never)]
fn update_by_spec(scheduler: &mut TickScheduler, node: &mut Node, slot: Slot) {
    let outcome = node.kind.update(
        node.state,
        node.inputs(),
        node.pending(scheduler.current_tick()),
    );
    if outcome.state != node.state {
        node.set_state(outcome.state);
    }
    if let Some(tick) = outcome.schedule {
        schedule_tick(scheduler, slot, node, tick.delay, tick.priority);
    }
    node.played_note |= outcome.plays_note;
}
