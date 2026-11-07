use crate::graph::NodeKind;
use mchprs_redstone::tick_queue::TickPriority;

/// The delays and priorities that ticks are scheduled with, one bit each. Ticks scheduled with the
/// same delay and priority during one tick run in the order they were scheduled.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct TickSlots(u64);

impl TickSlots {
    /// Where an update or a tick of a node of this kind schedules its own next tick
    pub fn of(kind: &NodeKind) -> TickSlots {
        match *kind {
            NodeKind::Repeater { delay, prioritized } => {
                let update = if prioritized {
                    TickPriority::Highest
                } else {
                    TickPriority::High
                };
                TickSlots::slot(delay, update).union(TickSlots::slot(delay, TickPriority::Higher))
            }
            NodeKind::Torch => TickSlots::slot(1, TickPriority::Normal),
            NodeKind::Comparator { prioritized, .. } => TickSlots::slot(
                1,
                if prioritized {
                    TickPriority::High
                } else {
                    TickPriority::Normal
                },
            ),
            NodeKind::Lamp => TickSlots::slot(2, TickPriority::Normal),
            NodeKind::Button { press_ticks } => TickSlots::slot(press_ticks, TickPriority::Normal),
            NodeKind::Lever
            | NodeKind::PressurePlate
            | NodeKind::Trapdoor
            | NodeKind::Wire
            | NodeKind::Constant
            | NodeKind::NoteBlock { .. } => TickSlots::default(),
        }
    }

    fn slot(delay: u8, priority: TickPriority) -> TickSlots {
        let index = delay as u32 * 4 + priority as u32;
        TickSlots(1u64.checked_shl(index).unwrap_or(u64::MAX))
    }

    pub fn union(self, other: TickSlots) -> TickSlots {
        TickSlots(self.0 | other.0)
    }

    pub fn intersects(self, other: TickSlots) -> bool {
        self.0 & other.0 != 0
    }
}
