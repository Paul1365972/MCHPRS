use super::{Circuit, CircuitError, Kind, Node, NodeError, PULSE_PRIORITY};
use crate::reference::settled_initial_state;

impl Circuit {
    pub(super) fn validate(&self) -> Result<(), CircuitError> {
        for (index, node) in self.nodes.iter().enumerate() {
            self.validate_links(node)
                .and_then(|()| validate_kind(node.kind))
                .and_then(|()| validate_reachable_state(node))
                .and_then(|()| self.validate_settled(index))
                .map_err(|problem| problem.at(index))?;
        }
        Ok(())
    }

    // The circuit settles every node on every clock instead of only when an input changes, which
    // is equivalent only for nodes that start settled.
    fn validate_settled(&self, index: usize) -> Result<(), NodeError> {
        if settled_initial_state(&self.nodes, index) == self.nodes[index].initial {
            Ok(())
        } else {
            Err(NodeError::Unsettled)
        }
    }

    fn validate_links(&self, node: &Node) -> Result<(), NodeError> {
        for link in node.inputs.iter().chain(&node.sides) {
            let source = self
                .nodes
                .get(link.source)
                .ok_or(NodeError::AbsentLinkSource(link.source))?;
            if link.attenuation > 15 {
                return Err(NodeError::Attenuation(link.attenuation));
            }
            if !source.kind.drives_links() {
                return Err(NodeError::UndrivenLink {
                    kind: source.kind,
                    link_source: link.source,
                });
            }
            if source.kind == Kind::Wire {
                return Err(NodeError::WireSource(link.source));
            }
        }
        if node.kind == Kind::Wire && !node.sides.is_empty() {
            return Err(NodeError::WireSides);
        }
        Ok(())
    }
}

fn validate_kind(kind: Kind) -> Result<(), NodeError> {
    match kind {
        Kind::Repeater { delay, .. } if !(1..=4).contains(&delay) => {
            Err(NodeError::RepeaterDelay(delay))
        }
        _ => Ok(()),
    }
}

fn validate_reachable_state(node: &Node) -> Result<(), NodeError> {
    let state = node.initial;
    if node.kind.has_boolean_power() && !matches!(state.strength.get(), 0 | 15) {
        return Err(NodeError::Strength {
            kind: node.kind,
            strength: state.strength,
        });
    }
    let has_sides = !node.sides.is_empty();
    if state.locked && !(matches!(node.kind, Kind::Repeater { .. }) && has_sides) {
        return Err(NodeError::Locked);
    }
    let Some(pending) = state.pending else {
        return Ok(());
    };
    let delay = node.kind.delay().ok_or(NodeError::Untimed(node.kind))?;
    if pending.delay > delay {
        return Err(NodeError::PendingDelayAbove {
            kind: node.kind,
            pending: pending.delay,
            delay,
        });
    }
    let scheduled = node.kind.priority(state.is_powered()) == Some(pending.priority)
        || node.kind.tracks_pulsing() && pending.priority == PULSE_PRIORITY;
    if !scheduled {
        return Err(NodeError::Priority {
            kind: node.kind,
            priority: pending.priority,
        });
    }
    Ok(())
}
