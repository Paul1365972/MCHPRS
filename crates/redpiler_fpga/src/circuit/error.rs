use super::Kind;
use mchprs_redpiler::compile_graph::{NodeType, SignalStrength};
use mchprs_world::TickPriority;
use std::num::NonZeroU8;

#[derive(Debug, thiserror::Error)]
pub(crate) enum CircuitError {
    #[error("a circuit needs at least one node")]
    Empty,
    #[error("{0} nodes exceed the 32-bit protocol")]
    TooManyNodes(usize),
    #[error("node {node}: {problem}")]
    Node { node: usize, problem: NodeError },
}

#[derive(Debug, thiserror::Error)]
pub(crate) enum NodeError {
    #[error("{0:?} is not supported")]
    Unsupported(NodeType),
    #[error("{0:?} cannot have a pending tick")]
    Untimed(Kind),
    #[error("pending delay {0} exceeds 15")]
    PendingDelay(u32),
    #[error("distinct pending ticks cannot be represented")]
    DistinctPendingTicks,
    #[error("marked pending without a pending tick")]
    MarkedPending,
    #[error("link source {0} is absent")]
    AbsentLinkSource(usize),
    #[error("attenuation {0} exceeds 15")]
    Attenuation(u8),
    #[error("{kind:?} source {link_source} drives nothing")]
    UndrivenLink { kind: Kind, link_source: usize },
    #[error("wire source {0} violates the frontend leaf-wire invariant")]
    WireSource(usize),
    #[error("wire cannot have side inputs")]
    WireSides,
    #[error("repeater delay {0} is not 1..4")]
    RepeaterDelay(u8),
    #[error("{kind:?} cannot have strength {strength}")]
    Strength {
        kind: Kind,
        strength: SignalStrength,
    },
    #[error("only a repeater with side inputs can be locked")]
    Locked,
    #[error("pending delay {pending} exceeds the delay {delay} of {kind:?}")]
    PendingDelayAbove {
        kind: Kind,
        pending: NonZeroU8,
        delay: NonZeroU8,
    },
    #[error("{kind:?} never schedules a tick at priority {priority:?}")]
    Priority { kind: Kind, priority: TickPriority },
    #[error("is not settled; update the world before compiling")]
    Unsettled,
}

impl NodeError {
    pub(super) fn at(self, node: usize) -> CircuitError {
        CircuitError::Node {
            node,
            problem: self,
        }
    }
}
