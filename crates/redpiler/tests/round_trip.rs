use mchprs_blocks::blocks::{ComparatorMode, Instrument};
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::BlockPos;
use mchprs_redpiler::circuit::{Link, NodeId};
use mchprs_redpiler::graph::{LinkKind, Node, NodeKind, NodeState, QueuedTick};
use mchprs_redpiler::nodes::{Changes, Snapshot};
use mchprs_redstone::tick_queue::{PendingTick, TickPriority};
use mchprs_simulation::ports::Role;
use serde::de::DeserializeOwned;
use serde::Serialize;
use smallvec::smallvec;
use std::fmt::Debug;

fn round_trip<T: Serialize + DeserializeOwned + PartialEq + Debug>(value: T) {
    let json = serde_json::to_string(&value).unwrap();
    assert_eq!(serde_json::from_str::<T>(&json).unwrap(), value, "{json}");
}

#[test]
fn graph_parts_round_trip() {
    let tick = PendingTick {
        delay: 2,
        priority: TickPriority::Higher,
    };
    let locked = NodeState {
        power: SignalStrength::Seven,
        repeater_locked: true,
        plate_touched: true,
    };

    round_trip(vec![
        NodeKind::Repeater {
            delay: 3,
            prioritized: true,
        },
        NodeKind::Torch,
        NodeKind::Comparator {
            mode: ComparatorMode::Subtract,
            far_input: Some(SignalStrength::Twelve),
            prioritized: true,
        },
        NodeKind::Lamp,
        NodeKind::Button { press_ticks: 10 },
        NodeKind::Lever,
        NodeKind::PressurePlate,
        NodeKind::Trapdoor,
        NodeKind::Wire,
        NodeKind::Constant,
        NodeKind::NoteBlock {
            instrument: Instrument::Bass,
            note: 24,
        },
    ]);
    round_trip(Node {
        kind: NodeKind::Torch,
        blocks: smallvec![(BlockPos::new(1, -2, 3), 4), (BlockPos::new(5, 6, -7), 8)],
        names: vec!["clock".to_owned(), "reset".to_owned()],
        state: locked,
        pending_tick: Some(QueuedTick { tick, order: 9 }),
        port: Some(Role::Output),
        observed: true,
    });
    round_trip(Link {
        source: NodeId::new(1),
        target: NodeId::new(2),
        kind: LinkKind::Side,
        weight: 14,
    });
    round_trip(Snapshot {
        states: vec![locked, NodeState::from_powered(true)],
        pending_ticks: vec![(NodeId::new(1), tick)],
    });
    round_trip(Changes {
        states: vec![(NodeId::new(0), locked)],
        played_note_blocks: vec![NodeId::new(1)],
    });
}
