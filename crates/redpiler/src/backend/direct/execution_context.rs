use super::node::NodeId;
use mchprs_world::TickPriority;
use std::mem;

#[derive(Default, Clone)]
pub struct Queues([Vec<NodeId>; ExecutionContext::NUM_PRIORITIES]);

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
pub struct ExecutionContext {
    queues_deque: [Queues; Self::NUM_QUEUES],
    pos: usize,
    changes: Vec<NodeId>,
}

impl ExecutionContext {
    const NUM_PRIORITIES: usize = 4;
    const NUM_QUEUES: usize = 16;

    pub(super) fn schedule_tick(&mut self, node: NodeId, delay: usize, priority: TickPriority) {
        self.queues_deque[(self.pos + delay) % Self::NUM_QUEUES].0[priority as usize].push(node);
    }

    pub(super) fn queues_this_tick(&mut self) -> Queues {
        self.pos = (self.pos + 1) % Self::NUM_QUEUES;
        mem::take(&mut self.queues_deque[self.pos])
    }

    pub(super) fn end_tick(&mut self, queues: Queues) {
        self.queues_deque[self.pos % Self::NUM_QUEUES] = queues;
    }

    fn priorities() -> [TickPriority; Self::NUM_PRIORITIES] {
        [
            TickPriority::Highest,
            TickPriority::Higher,
            TickPriority::High,
            TickPriority::Normal,
        ]
    }

    pub(super) fn has_pending_ticks(&self) -> bool {
        for queues in &self.queues_deque {
            for queue in &queues.0 {
                if !queue.is_empty() {
                    return true;
                }
            }
        }
        false
    }

    pub(super) fn push_change(&mut self, node_id: NodeId) {
        self.changes.push(node_id);
    }

    pub(super) fn drain_changes(&mut self) -> impl Iterator<Item = NodeId> + '_ {
        self.changes.drain(..)
    }

    pub(super) fn drain_scheduled_ticks(
        &mut self,
    ) -> impl Iterator<Item = (usize, NodeId, TickPriority)> + '_ {
        let pos = self.pos;
        self.queues_deque
            .iter_mut()
            .enumerate()
            .flat_map(move |(queue_idx, queues)| {
                let delay = (queue_idx + Self::NUM_QUEUES - pos) % Self::NUM_QUEUES;

                queues
                    .0
                    .iter_mut()
                    .zip(Self::priorities())
                    .flat_map(move |(queue, priority)| {
                        queue
                            .drain(..)
                            .map(move |node_id| (delay, node_id, priority))
                    })
            })
    }
}
