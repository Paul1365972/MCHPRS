use mchprs_blocks::{blocks::Block, BlockPos};
use mchprs_world::{TickPriority, World};
use std::mem;
use tracing::warn;

use super::node::NodeId;

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
}

impl ExecutionContext {
    const NUM_PRIORITIES: usize = 4;
    const NUM_QUEUES: usize = 16;

    pub(super) fn reset<W: World>(
        &mut self,
        world: &mut W,
        blocks: &[impl AsRef<[(BlockPos, Block)]>],
    ) {
        for (idx, queues) in self.queues_deque.iter().enumerate() {
            let delay = if self.pos >= idx {
                idx + Self::NUM_QUEUES
            } else {
                idx
            } - self.pos;
            for (entries, priority) in queues.0.iter().zip(Self::priorities()) {
                for node in entries {
                    let node_blocks = blocks[node.index()].as_ref();
                    if node_blocks.is_empty() {
                        warn!("Cannot schedule tick for node {:?} because block information is missing", node);
                        continue;
                    };
                    for (pos, _) in node_blocks.iter().copied() {
                        world.schedule_tick(pos, delay as u32, priority);
                    }
                }
            }
        }
        for queues in self.queues_deque.iter_mut() {
            for queue in queues.0.iter_mut() {
                queue.clear();
            }
        }
    }

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

}
