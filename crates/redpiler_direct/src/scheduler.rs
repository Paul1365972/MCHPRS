use crate::node::NodeId;
use mchprs_redpiler::circuit;
use mchprs_redpiler::graph::PendingTick;
use mchprs_redstone::tick_queue::TickPriority;
use std::mem;

#[derive(Default, Clone)]
pub struct Queues([Vec<NodeId>; TickScheduler::NUM_PRIORITIES]);

impl Queues {
    #[inline(always)]
    pub fn drain_each<F: FnMut(NodeId)>(&mut self, mut f: F) {
        for q in self.0.iter_mut() {
            for n in q.iter() {
                f(*n);
            }
            q.clear();
        }
    }
}

#[derive(Default)]
pub struct TickScheduler {
    queues_deque: [Queues; Self::NUM_QUEUES],
    tick: u32,
}

impl TickScheduler {
    const NUM_PRIORITIES: usize = 4;
    const NUM_QUEUES: usize = 16;

    fn queue_index(tick: u32) -> usize {
        tick as usize % Self::NUM_QUEUES
    }

    pub fn current_tick(&self) -> u32 {
        self.tick
    }

    pub fn schedule_tick(&mut self, node: NodeId, delay: u32, priority: TickPriority) -> u32 {
        let tick = self.tick.wrapping_add(delay);
        self.queues_deque[Self::queue_index(tick)].0[priority as usize].push(node);
        tick
    }

    pub fn queues_this_tick(&mut self) -> Queues {
        self.tick = self.tick.wrapping_add(1);
        mem::take(&mut self.queues_deque[Self::queue_index(self.tick)])
    }

    pub fn end_tick(&mut self, queues: Queues) {
        self.queues_deque[Self::queue_index(self.tick)] = queues;
    }

    fn priorities() -> [TickPriority; Self::NUM_PRIORITIES] {
        [
            TickPriority::Highest,
            TickPriority::Higher,
            TickPriority::High,
            TickPriority::Normal,
        ]
    }

    pub fn has_pending_ticks(&self) -> bool {
        self.queues_deque
            .iter()
            .any(|queues| queues.0.iter().any(|queue| !queue.is_empty()))
    }

    pub fn pending_ticks(&self) -> impl Iterator<Item = (NodeId, PendingTick)> + '_ {
        (1..=Self::NUM_QUEUES as u32).flat_map(move |delay| {
            let queues = &self.queues_deque[Self::queue_index(self.tick.wrapping_add(delay))];
            queues
                .0
                .iter()
                .zip(Self::priorities())
                .flat_map(move |(queue, priority)| {
                    queue
                        .iter()
                        .map(move |&node| (node, PendingTick { delay, priority }))
                })
        })
    }

    pub fn clear(&mut self) {
        for queues in &mut self.queues_deque {
            for queue in &mut queues.0 {
                queue.clear();
            }
        }
    }
}

pub fn check_delays(pending_ticks: &[(circuit::NodeId, PendingTick)]) -> Result<(), String> {
    for &(node, tick) in pending_ticks {
        if tick.delay as usize > TickScheduler::NUM_QUEUES {
            return Err(format!(
                "pending tick of node {} is {} ticks away (at most {} supported)",
                node.index(),
                tick.delay,
                TickScheduler::NUM_QUEUES
            ));
        }
    }
    Ok(())
}
