use crate::circuit::NodeId;
use crate::graph::NodeState;
use mchprs_redstone::tick_queue::PendingTick;
use mchprs_simulation::Space;
use serde::{Deserialize, Serialize};

/// A circuit, addressed by node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Nodes;

impl Space for Nodes {
    type Address = NodeId;
    type Changes = Changes;
    type State = Snapshot;
}

/// A backend that sees every tick, such as `direct`, also lists a node that changed and is back as
/// it was.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Changes {
    pub states: Vec<(NodeId, NodeState)>,
    pub played_note_blocks: Vec<NodeId>,
}

impl Extend<Changes> for Changes {
    fn extend<I: IntoIterator<Item = Changes>>(&mut self, later: I) {
        for mut changes in later {
            self.states.append(&mut changes.states);
            self.played_note_blocks
                .append(&mut changes.played_note_blocks);
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Snapshot {
    pub states: Vec<NodeState>,
    pub pending_ticks: Vec<(NodeId, PendingTick)>,
}
