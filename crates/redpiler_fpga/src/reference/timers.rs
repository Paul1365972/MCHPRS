use crate::circuit::PendingTick;
use mchprs_world::TickPriority;
use std::num::NonZeroU8;

// Each slot must hold a single deadline, so the slot count must exceed the longest delay.
const SLOT_COUNT: usize = 16;

fn slot(deadline: u128) -> usize {
    (deadline % SLOT_COUNT as u128) as usize
}

#[derive(Clone, Copy)]
struct Timer {
    deadline: u128,
    position: Option<usize>,
    priority: TickPriority,
}

pub(super) struct Timers {
    pending: Vec<Option<Timer>>,
    queue: [Vec<usize>; SLOT_COUNT],
}

impl Timers {
    pub(super) fn new(count: usize) -> Self {
        Self {
            pending: vec![None; count],
            queue: Default::default(),
        }
    }

    pub(super) fn remaining(&self, index: usize, ticks: u64) -> Option<PendingTick> {
        let timer = self.pending[index]?;
        Some(PendingTick {
            delay: NonZeroU8::new((timer.deadline - u128::from(ticks)) as u8)?,
            priority: timer.priority,
        })
    }

    pub(super) fn is_pending(&self, index: usize) -> bool {
        self.pending[index].is_some()
    }

    pub(super) fn next_deadline(&self) -> Option<u128> {
        self.queue
            .iter()
            .filter_map(|slot| {
                slot.first()
                    .map(|index| self.pending[*index].unwrap().deadline)
            })
            .min()
    }

    pub(super) fn freeze(&mut self, deadline: u128, phases: &mut [Vec<usize>; 4]) {
        for index in self.queue[slot(deadline)].drain(..) {
            let timer = self.pending[index].as_mut().unwrap();
            timer.position = None;
            phases[timer.priority as usize].push(index);
        }
    }

    pub(super) fn consume(&mut self, index: usize) {
        self.pending[index] = None;
    }

    pub(super) fn schedule(&mut self, index: usize, deadline: u128, priority: TickPriority) {
        if let Some(Timer {
            deadline: previous,
            position: Some(position),
            ..
        }) = self.pending[index]
        {
            let due = &mut self.queue[slot(previous)];
            due.swap_remove(position);
            if let Some(moved) = due.get(position) {
                self.pending[*moved].as_mut().unwrap().position = Some(position);
            }
        }
        let due = &mut self.queue[slot(deadline)];
        self.pending[index] = Some(Timer {
            deadline,
            position: Some(due.len()),
            priority,
        });
        due.push(index);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frozen(timers: &mut Timers, deadline: u128) -> [Vec<usize>; 4] {
        let mut phases = Default::default();
        timers.freeze(deadline, &mut phases);
        phases
    }

    #[test]
    fn due_timers_freeze_into_their_priority_phases() {
        let mut timers = Timers::new(3);
        timers.schedule(0, 5, TickPriority::Normal);
        timers.schedule(1, 5, TickPriority::Highest);
        timers.schedule(2, 6, TickPriority::High);
        assert_eq!(timers.next_deadline(), Some(5));
        assert_eq!(frozen(&mut timers, 5), [vec![1], vec![], vec![], vec![0]]);
        timers.consume(0);
        timers.consume(1);
        assert!(!timers.is_pending(0));
        assert_eq!(timers.next_deadline(), Some(6));
    }

    #[test]
    fn remaining_counts_down_to_the_deadline() {
        let mut timers = Timers::new(1);
        timers.schedule(0, 18, TickPriority::Higher);
        assert_eq!(
            timers.remaining(0, 15),
            Some(PendingTick {
                delay: NonZeroU8::new(3).unwrap(),
                priority: TickPriority::Higher,
            })
        );
        assert_eq!(timers.remaining(0, 18), None);
    }

    #[test]
    fn rescheduling_moves_a_timer_out_of_its_slot() {
        let mut timers = Timers::new(3);
        for index in 0..3 {
            timers.schedule(index, 3, TickPriority::Normal);
        }
        timers.schedule(0, 4, TickPriority::Normal);
        timers.schedule(2, 4, TickPriority::Normal);
        assert_eq!(frozen(&mut timers, 3), [vec![], vec![], vec![], vec![1]]);
        assert_eq!(frozen(&mut timers, 4), [vec![], vec![], vec![], vec![0, 2]]);
    }
}
