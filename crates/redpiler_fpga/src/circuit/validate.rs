use super::{invalid, Circuit, Kind, Node, PULSE_PRIORITY};
use std::io;

impl Circuit {
    pub(super) fn validate(&self) -> io::Result<()> {
        for (index, node) in self.nodes.iter().enumerate() {
            self.validate_links(node)
                .and_then(|()| validate_kind(node.kind))
                .and_then(|()| validate_reachable_state(node))
                .map_err(|problem| invalid(format!("node {index}: {problem}")))?;
        }
        Ok(())
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
        Kind::Comparator {
            far_input: Some(strength),
            ..
        } if strength > 15 => Err(format!("comparator far input {strength} exceeds 15")),
        _ => Ok(()),
    }
}

fn validate_reachable_state(node: &Node) -> Result<(), String> {
    let state = node.initial;
    if state.strength > 15 {
        return Err(format!("strength {} exceeds 15", state.strength));
    }
    if node.kind.has_boolean_power() && !matches!(state.strength, 0 | 15) {
        return Err(format!(
            "{:?} cannot have strength {}",
            node.kind, state.strength
        ));
    }
    let has_sides = !node.sides.is_empty();
    if state.locked && !(matches!(node.kind, Kind::Repeater { .. }) && has_sides) {
        return Err("only a repeater with side inputs can be locked".to_owned());
    }
    if state.delay == 0 {
        return Ok(());
    }
    let delay = node
        .kind
        .delay()
        .ok_or_else(|| format!("{:?} cannot have a pending tick", node.kind))?;
    if state.delay > delay {
        return Err(format!(
            "pending delay {} exceeds the delay {delay} of {:?}",
            state.delay, node.kind
        ));
    }
    let scheduled = node.kind.priority(state.is_powered()) == Some(state.priority)
        || node.kind.has_pulse_countdown() && state.priority == PULSE_PRIORITY;
    if !scheduled {
        return Err(format!(
            "{:?} never schedules a tick at priority {}",
            node.kind, state.priority
        ));
    }
    Ok(())
}
