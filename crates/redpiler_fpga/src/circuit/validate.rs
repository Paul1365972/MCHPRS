use super::{invalid, Circuit, Kind, Node, PULSE_PRIORITY};
use crate::reference::settled_initial_state;
use std::io;

impl Circuit {
    pub(super) fn validate(&self) -> io::Result<()> {
        for (index, node) in self.nodes.iter().enumerate() {
            self.validate_links(node)
                .and_then(|()| validate_kind(node.kind))
                .and_then(|()| validate_reachable_state(node))
                .and_then(|()| self.validate_settled(index))
                .map_err(|problem| invalid(format!("node {index}: {problem}")))?;
        }
        Ok(())
    }

    // The circuit settles every node on every clock instead of only when an input changes, which
    // is equivalent only for nodes that start settled.
    fn validate_settled(&self, index: usize) -> Result<(), String> {
        if settled_initial_state(&self.nodes, index) == self.nodes[index].initial {
            Ok(())
        } else {
            Err("is not settled; update the world before compiling".to_owned())
        }
    }

    fn validate_links(&self, node: &Node) -> Result<(), String> {
        for link in node.inputs.iter().chain(&node.sides) {
            let source = self.nodes.get(link.source).ok_or("link source is absent")?;
            if link.attenuation > 15 {
                return Err("attenuation exceeds 15".to_owned());
            }
            if !source.kind.drives_links() {
                return Err(format!(
                    "{:?} source {} drives nothing",
                    source.kind, link.source
                ));
            }
            if source.kind == Kind::Wire {
                return Err(format!(
                    "wire source {} violates the frontend leaf-wire invariant",
                    link.source
                ));
            }
        }
        if node.kind == Kind::Wire && !node.sides.is_empty() {
            return Err("wire cannot have side inputs".to_owned());
        }
        Ok(())
    }
}

fn validate_kind(kind: Kind) -> Result<(), String> {
    match kind {
        Kind::Repeater { delay, .. } if !(1..=4).contains(&delay) => {
            Err(format!("repeater delay {delay} is not 1..4"))
        }
        _ => Ok(()),
    }
}

fn validate_reachable_state(node: &Node) -> Result<(), String> {
    let state = node.initial;
    if node.kind.has_boolean_power() && !matches!(state.strength.get(), 0 | 15) {
        return Err(format!(
            "{:?} cannot have strength {}",
            node.kind, state.strength
        ));
    }
    let has_sides = !node.sides.is_empty();
    if state.locked && !(matches!(node.kind, Kind::Repeater { .. }) && has_sides) {
        return Err("only a repeater with side inputs can be locked".to_owned());
    }
    let Some(pending) = state.pending else {
        return Ok(());
    };
    let delay = node
        .kind
        .delay()
        .ok_or_else(|| format!("{:?} cannot have a pending tick", node.kind))?;
    if pending.delay > delay {
        return Err(format!(
            "pending delay {} exceeds the delay {delay} of {:?}",
            pending.delay, node.kind
        ));
    }
    let scheduled = node.kind.priority(state.is_powered()) == Some(pending.priority)
        || node.kind.has_pulse_countdown() && pending.priority == PULSE_PRIORITY;
    if !scheduled {
        return Err(format!(
            "{:?} never schedules a tick at priority {:?}",
            node.kind, pending.priority
        ));
    }
    Ok(())
}
