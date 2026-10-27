mod common;

use common::{graph, node_named, Run};
use mchprs_redpiler::graph::{Direction, NodeState};
use mchprs_redstone::input::Input;
use mchprs_redstone::tick_queue::{PendingTick, TickPriority};
use mchprs_simulation::Simulation;

impl Run {
    fn toggle(&mut self, name: &str) {
        let toggled = self.engine.drive(self.node(name), Input::Toggle);
        toggled.expect("the node takes a toggle");
    }

    fn state(&mut self, name: &str) -> NodeState {
        self.engine.complete_snapshot().states[self.node(name).index()]
    }

    fn pending_tick(&mut self, name: &str) -> Option<PendingTick> {
        let node = self.node(name);
        let mut pending_ticks = self.engine.complete_snapshot().pending_ticks.into_iter();
        let (_, tick) = pending_ticks.find(|&(pending, _)| pending == node)?;
        Some(tick)
    }
}

const NEXT_TICK: PendingTick = PendingTick {
    delay: 1,
    priority: TickPriority::Normal,
};

/// Under `direct` the subtract comparator pulses only if its tick was scheduled first.
#[test]
fn ticks_pending_at_the_start_run_at_once_whichever_was_scheduled_first() {
    for (compare_order, subtract_order) in [(0, 1), (1, 0)] {
        let mut circuit = graph(include_str!("circuits/due_together.ril"));
        for (name, order) in [("compare", compare_order), ("subtract", subtract_order)] {
            let node = node_named(&circuit, name);
            circuit[node]
                .pending_tick
                .as_mut()
                .expect("a pending tick")
                .order = order;
        }
        let mut run = Run::new(circuit);
        run.tick();
        assert_eq!(run.state("compare"), NodeState::from_powered(true));
        assert_eq!(run.state("subtract"), NodeState::from_powered(true));
        assert_eq!(run.pending_tick("subtract"), Some(NEXT_TICK));
        run.tick();
        assert_eq!(run.state("subtract"), NodeState::from_powered(false));
        assert_eq!(run.pending_tick("subtract"), None);
    }
}

/// The torch was scheduled in an earlier priority, so under `direct` and in vanilla it ticks first
/// and the comparator stays off.
#[test]
fn comparator_due_with_the_tick_that_blocks_it_pulses() {
    let circuit = graph(include_str!("circuits/gate_closes.ril"));
    let mut run = Run::new(circuit);
    run.toggle("lever");
    run.tick();
    run.tick();
    assert_eq!(run.state("pass"), NodeState::from_powered(true));
    run.tick();
    assert_eq!(run.state("pass"), NodeState::from_powered(false));
}

/// Under `direct` the repeater keeps a tick pending if the torch ticks first.
#[test]
fn two_ticks_into_one_node_update_it_once_with_both_changes() {
    for reversed in [false, true] {
        let mut circuit = graph(include_str!("circuits/two_ticks_into_one_node.ril"));
        let lever = node_named(&circuit, "lever");
        if reversed {
            circuit.sort_edges_by_key(lever, Direction::Outgoing, |_, edge| {
                std::cmp::Reverse(edge.id().index())
            });
        }
        let mut run = Run::new(circuit);
        run.toggle("lever");
        run.tick();
        assert_eq!(run.state("repeater"), NodeState::repeater(true, true));
        assert_eq!(run.pending_tick("repeater"), None);
    }
}

/// Under `direct` the comparator schedules a tick that changes nothing if its rear link is
/// updated first.
#[test]
fn change_into_rear_and_side_updates_once_with_both() {
    for reversed in [false, true] {
        let mut circuit = graph(include_str!("circuits/rear_and_side.ril"));
        let lever = node_named(&circuit, "lever");
        if reversed {
            circuit.sort_edges_by_key(lever, Direction::Outgoing, |_, edge| {
                std::cmp::Reverse(edge.id().index())
            });
        }
        let mut run = Run::new(circuit);
        run.toggle("lever");
        assert_eq!(run.state("comparator"), NodeState::from_powered(false));
        assert_eq!(run.pending_tick("comparator"), None);
    }
}

#[test]
fn node_updated_before_its_tick_runs_sees_it_due_this_tick() {
    let circuit = graph(include_str!("circuits/due_this_tick.ril"));
    let mut run = Run::new(circuit);
    run.tick();
    assert_eq!(run.state("lamp"), NodeState::from_powered(false));
    let after_two_ticks = PendingTick {
        delay: 2,
        priority: TickPriority::Normal,
    };
    assert_eq!(run.pending_tick("lamp"), Some(after_two_ticks));
    assert_eq!(run.state("torch"), NodeState::from_powered(true));
    assert_eq!(run.pending_tick("torch"), None);
}
