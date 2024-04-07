use mchprs_world::TickPriority;

use crate::redpiler::backend::nodes::NodeId;

use super::TickScheduler;
use super::*;

pub(super) fn update_node_default_digital(
    scheduler: &mut TickScheduler,
    node_id: NodeId,
    node: &mut Node,
) {
    if node.pending_tick {
        return;
    }
    let should_be_powered = get_bool_input(node);
    match node.ty {
        NodeType::Repeater {
            delay,
            facing_diode,
        } => {
            let locked = get_bool_side(node);

            if !node.pending_tick && !locked && should_be_powered != node.powered {
                let priority = if facing_diode {
                    TickPriority::Highest
                } else if !should_be_powered {
                    TickPriority::Higher
                } else {
                    TickPriority::High
                };
                schedule_tick(
                    scheduler,
                    NodeIdWithData::new(node_id),
                    node,
                    delay as usize,
                    priority,
                );
            }
        }
        NodeType::Torch => {
            if !node.pending_tick && should_be_powered == node.powered {
                schedule_tick(
                    scheduler,
                    NodeIdWithData::new(node_id),
                    node,
                    1,
                    TickPriority::Normal,
                );
            }
        }
        NodeType::Lamp => {
            if should_be_powered && !node.powered {
                set_node(node, true);
            } else if !should_be_powered && node.powered {
                schedule_tick(
                    scheduler,
                    NodeIdWithData::new(node_id),
                    node,
                    2,
                    TickPriority::Normal,
                );
            }
        }
        NodeType::Trapdoor => {
            set_node(node, should_be_powered);
        }
        _ => unsafe { std::hint::unreachable_unchecked() },
    }
}

pub(super) fn update_node_default_analog(
    scheduler: &mut TickScheduler,
    outputs: &mut [Sender<UpdateMessage>],
    node_id: NodeId,
    node: &mut Node,
) {
    let input_power = get_analog_input(node);
    match node.ty {
        NodeType::Comparator {
            mode,
            far_input,
            facing_diode,
        } => {
            if node.pending_tick {
                return;
            }
            let mut input_power = get_analog_input(node);
            let side_input_power = get_analog_side(node);
            if let Some(far_override) = far_input {
                if input_power < 15 {
                    input_power = far_override.get();
                }
            }
            let old_strength = node.output_power;
            let output_power = calculate_comparator_output(mode, input_power, side_input_power);
            if output_power != old_strength {
                let priority = if facing_diode {
                    TickPriority::High
                } else {
                    TickPriority::Normal
                };
                schedule_tick(scheduler, NodeIdWithData::new(node_id), node, 1, priority);
            }
        }
        NodeType::Wire => {
            node.output_power = input_power;
            node.changed = true;
        }
        NodeType::ExternalOutput {
            output_index,
            target_id,
        } => {
            node.output_power = input_power;
            node.changed = true;

            let output = unsafe { outputs.get_unchecked_mut(output_index as usize) };
            output.send_unsafe(UpdateMessage::Update {
                node_id: target_id,
                signal_strength: input_power,
            });
        }
        NodeType::ComparatorLine { .. } => {
            if !node.pending_tick {
                schedule_tick(
                    scheduler,
                    NodeIdWithData::new_with_data(node_id, false, 0),
                    node,
                    1,
                    TickPriority::Normal,
                );
            }
        }
        _ => unsafe { std::hint::unreachable_unchecked() },
    }
}

pub(super) fn update_node_side_digital(
    scheduler: &mut TickScheduler,
    node_id: NodeId,
    node: &mut Node,
) {
    match node.ty {
        NodeType::Repeater {
            delay,
            facing_diode,
        } => {
            let locked = get_bool_side(node);
            node.changed = true;

            let should_be_powered = get_bool_input(node);
            if !locked && !node.pending_tick && should_be_powered != node.powered {
                let priority = if facing_diode {
                    TickPriority::Highest
                } else if !should_be_powered {
                    TickPriority::Higher
                } else {
                    TickPriority::High
                };
                schedule_tick(
                    scheduler,
                    NodeIdWithData::new(node_id),
                    node,
                    delay as usize,
                    priority,
                );
            }
        }
        _ => unsafe { std::hint::unreachable_unchecked() },
    }
}

pub(super) fn update_node_side_analog(
    scheduler: &mut TickScheduler,
    node_id: NodeId,
    node: &mut Node,
) {
    match node.ty {
        NodeType::Comparator {
            mode,
            far_input,
            facing_diode,
        } => {
            if node.pending_tick {
                return;
            }
            let mut input_power = get_analog_input(node);
            let side_input_power = get_analog_side(node);
            if let Some(far_override) = far_input {
                if input_power < 15 {
                    input_power = far_override.get();
                }
            }
            let old_strength = node.output_power;
            let output_power = calculate_comparator_output(mode, input_power, side_input_power);
            if output_power != old_strength {
                let priority = if facing_diode {
                    TickPriority::High
                } else {
                    TickPriority::Normal
                };
                schedule_tick(scheduler, NodeIdWithData::new(node_id), node, 1, priority);
            }
        }
        _ => unsafe { std::hint::unreachable_unchecked() },
    }
}
