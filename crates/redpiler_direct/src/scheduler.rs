use crate::node::Slot;
use mchprs_redpiler::circuit::NodeId;
use mchprs_redstone::tick_queue::{PendingTick, TickPriority};
use std::mem;

// The tick loop is measurably slower without this alignment.
#[repr(align(16))]
#[derive(Debug, Default, Clone)]
pub struct Queues([Vec<Slot>; TickPriority::ALL.len()]);

impl Queues {
    #[inline(always)]
    pub fn for_each<F: FnMut(Slot)>(&self, mut f: F) {
        for queue in self.0.iter() {
            for node in queue.iter() {
                f(*node);
            }
        }
    }
}

#[derive(Debug, Default)]
pub struct TickScheduler {
    ring: [Queues; Self::RING_LEN],
    tick: u32,
}

impl TickScheduler {
    const RING_LEN: usize = 16;

    fn queue_index(tick: u32) -> usize {
        tick as usize % Self::RING_LEN
    }

    pub fn current_tick(&self) -> u32 {
        self.tick
    }

    #[inline(always)]
    pub fn schedule_tick(&mut self, node: Slot, delay: u32, priority: TickPriority) -> u32 {
        let tick = self.tick.wrapping_add(delay);
        self.ring[Self::queue_index(tick)].0[priority as usize].push(node);
        tick
    }

    pub fn queues_this_tick(&mut self) -> Queues {
        self.tick = self.tick.wrapping_add(1);
        mem::take(&mut self.ring[Self::queue_index(self.tick)])
    }

    pub fn end_tick(&mut self, queues: Queues) {
        let slot = &mut self.ring[Self::queue_index(self.tick)];
        *slot = queues;
        for queue in &mut slot.0 {
            queue.clear();
        }
    }

    pub fn has_pending_ticks(&self) -> bool {
        self.ring
            .iter()
            .any(|queues| queues.0.iter().any(|queue| !queue.is_empty()))
    }

    pub fn pending_ticks(&self) -> impl Iterator<Item = (Slot, PendingTick)> + '_ {
        (1..=Self::RING_LEN as u32).flat_map(move |delay| {
            let queues = &self.ring[Self::queue_index(self.tick.wrapping_add(delay))];
            queues
                .0
                .iter()
                .zip(TickPriority::ALL)
                .flat_map(move |(queue, priority)| {
                    queue
                        .iter()
                        .map(move |&node| (node, PendingTick { delay, priority }))
                })
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error(
    "pending tick of node {} is {delay} ticks away (at most {} supported)",
    .node.index(),
    TickScheduler::RING_LEN
)]
pub struct DelayTooLong {
    node: NodeId,
    delay: u32,
}

pub fn check_delays(pending_ticks: &[(NodeId, PendingTick)]) -> Result<(), DelayTooLong> {
    for &(node, tick) in pending_ticks {
        if tick.delay as usize > TickScheduler::RING_LEN {
            return Err(DelayTooLong {
                node,
                delay: tick.delay,
            });
        }
    }
    Ok(())
}
