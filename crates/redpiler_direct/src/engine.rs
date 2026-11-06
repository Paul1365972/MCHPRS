mod tick;
mod update;

use crate::compile;
use crate::node::{ForwardLinks, Node, NodeId, NodeInput, NodeKind, Nodes};
use crate::scheduler::{check_delays, TickScheduler};
use mchprs_blocks::blocks::ComparatorMode;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redpiler::backend::BuildError;
use mchprs_redpiler::circuit::{self, Circuit};
use mchprs_redpiler::engine::{Changes, Drive, Engine, EngineError, Input, Snapshot};
use mchprs_redpiler::graph::PendingTick;
use mchprs_redstone::tick_queue::TickPriority;
use std::mem;

struct NoteActivations {
    counts: Box<[u32]>,
    played: Vec<NodeId>,
}

impl NoteActivations {
    fn new(len: usize) -> NoteActivations {
        NoteActivations {
            counts: vec![0; len].into_boxed_slice(),
            played: Vec::new(),
        }
    }

    fn record(&mut self, node: NodeId) {
        let count = &mut self.counts[node.index()];
        if *count == 0 {
            self.played.push(node);
        }
        *count = count.saturating_add(1);
    }

    fn drain(&mut self) -> impl Iterator<Item = (NodeId, u32)> + '_ {
        let counts = &mut self.counts;
        self.played
            .drain(..)
            .map(move |node| (node, mem::take(&mut counts[node.index()])))
    }

    fn clear(&mut self) {
        self.drain().for_each(drop);
    }
}

pub struct DirectEngine {
    nodes: Nodes,
    visible_nodes: Box<[NodeId]>,
    forward_links: ForwardLinks,
    scheduler: TickScheduler,
    notes: NoteActivations,
    initial: Snapshot,
}

impl DirectEngine {
    pub fn new(circuit: &Circuit) -> Result<DirectEngine, BuildError> {
        let (nodes, forward_links) = compile::lower(circuit)?;
        let initial = circuit.initial();
        check_delays(&initial.pending_ticks).map_err(BuildError::Unsupported)?;
        let visible_nodes = (0..nodes.len())
            .filter(|&index| nodes[index].visible)
            .map(|index| NodeId::new(index, nodes.len()))
            .collect();
        let mut engine = DirectEngine {
            nodes: Nodes::new(nodes),
            visible_nodes,
            forward_links,
            scheduler: TickScheduler::default(),
            notes: NoteActivations::new(circuit.len()),
            initial: Snapshot::default(),
        };
        engine.rebuild_inputs();
        engine.schedule(&initial.pending_ticks);
        engine.initial = initial;
        Ok(engine)
    }

    fn rebuild_inputs(&mut self) {
        for node in self.nodes.inner_mut() {
            node.default_inputs = NodeInput::new();
            node.side_inputs = NodeInput::new();
        }
        for source in 0..self.nodes.inner().len() {
            let power = self.nodes.inner()[source].power;
            let links = self.nodes.inner()[source].fwd_link_range.clone();
            for link in self.forward_links.get(&links) {
                let target = &mut self.nodes[link.node()];
                let inputs = if link.side() {
                    &mut target.side_inputs
                } else {
                    &mut target.default_inputs
                };
                inputs.update_power(SignalStrength::Zero, power.saturating_sub(link.weight()));
            }
        }
    }

    fn schedule(&mut self, pending_ticks: &[(circuit::NodeId, PendingTick)]) {
        for &(node, tick) in pending_ticks {
            let node_id = self.nodes.get(node.index());
            let node = &mut self.nodes[node_id];
            schedule_tick(
                &mut self.scheduler,
                node_id,
                node,
                tick.delay,
                tick.priority,
            );
        }
    }

    #[inline(always)]
    fn tick(&mut self) {
        let mut queues = self.scheduler.queues_this_tick();
        queues.drain_each(|node_id| {
            self.tick_node(node_id);
        });

        self.scheduler.end_tick(queues);
    }

    fn set_power_and_propagate(&mut self, node_id: NodeId, power: SignalStrength) {
        let node = &mut self.nodes[node_id];
        let old_power = node.power;
        node.set_power(power);

        for forward_link in self.forward_links.get(&node.fwd_link_range) {
            let side = forward_link.side();
            let weight = forward_link.weight();
            let update = forward_link.node();

            let update_ref = &mut self.nodes[update];
            let inputs = if side {
                &mut update_ref.side_inputs
            } else {
                &mut update_ref.default_inputs
            };

            let old_input = old_power.saturating_sub(weight);
            let new_input = power.saturating_sub(weight);

            if old_input == new_input {
                continue;
            }

            inputs.update_power(old_input, new_input);

            update::update_node(
                &mut self.scheduler,
                &mut self.notes,
                &mut self.nodes,
                update,
            );
        }
    }
}

impl Engine for DirectEngine {
    fn drive(&mut self, drive: Drive) -> Result<(), EngineError> {
        let node_id = self.nodes.get(drive.node().index());
        match drive.input() {
            Input::Press => {
                let node = &mut self.nodes[node_id];
                if let NodeKind::Button { press_ticks } = node.kind
                    && !node.is_powered()
                {
                    if !node.has_pending_tick() {
                        schedule_tick(
                            &mut self.scheduler,
                            node_id,
                            node,
                            press_ticks.into(),
                            TickPriority::Normal,
                        );
                    }
                    self.set_power_and_propagate(node_id, SignalStrength::Fifteen);
                }
            }
            Input::Set(power) => {
                if self.nodes[node_id].power != power {
                    self.set_power_and_propagate(node_id, power);
                }
            }
            Input::Toggle => {
                let power = (!self.nodes[node_id].is_powered()).into();
                self.set_power_and_propagate(node_id, power);
            }
        }
        Ok(())
    }

    fn run(&mut self, ticks: u64) -> Result<(), EngineError> {
        for _ in 0..ticks {
            self.tick();
        }
        Ok(())
    }

    fn settle(&mut self, limit: u64) -> Result<Option<u64>, EngineError> {
        for ticks in 0..limit {
            if !self.scheduler.has_pending_ticks() {
                return Ok(Some(ticks));
            }
            self.tick();
        }
        Ok((!self.scheduler.has_pending_ticks()).then_some(limit))
    }

    fn changes(&mut self, changes: &mut Changes) -> Result<(), EngineError> {
        for &node_id in &self.visible_nodes {
            let node = &mut self.nodes[node_id];
            if node.changed {
                node.changed = false;
                changes
                    .states
                    .push((circuit::NodeId::new(node_id.index()), node.state()));
            }
        }
        for (node, count) in self.notes.drain() {
            if self.nodes[node].visible {
                changes
                    .notes
                    .push((circuit::NodeId::new(node.index()), count));
            }
        }
        Ok(())
    }

    fn snapshot(&mut self) -> Result<Snapshot, EngineError> {
        Ok(Snapshot {
            states: self.nodes.inner().iter().map(Node::state).collect(),
            pending_ticks: self
                .scheduler
                .pending_ticks()
                .map(|(node, tick)| (circuit::NodeId::new(node.index()), tick))
                .collect(),
        })
    }

    fn restore(&mut self, snapshot: &Snapshot) -> Result<(), EngineError> {
        if snapshot.states.len() != self.nodes.inner().len() {
            return Err(format!(
                "snapshot has {} node states (expected {})",
                snapshot.states.len(),
                self.nodes.inner().len()
            )
            .into());
        }
        check_delays(&snapshot.pending_ticks)?;
        for (node, state) in self.nodes.inner_mut().iter_mut().zip(&snapshot.states) {
            node.power = state.power;
            node.repeater_locked = state.repeater_locked;
            node.due_tick = None;
            node.changed = true;
        }
        self.rebuild_inputs();
        self.scheduler.clear();
        self.notes.clear();
        self.schedule(&snapshot.pending_ticks);
        Ok(())
    }

    fn reset(&mut self) -> Result<(), EngineError> {
        let initial = mem::take(&mut self.initial);
        let reset = self.restore(&initial);
        self.initial = initial;
        reset
    }
}

fn schedule_tick(
    scheduler: &mut TickScheduler,
    node_id: NodeId,
    node: &mut Node,
    delay: u32,
    priority: TickPriority,
) {
    node.due_tick = Some(scheduler.schedule_tick(node_id, delay, priority));
}

#[inline]
fn comparator_output_power(
    node: &Node,
    mode: ComparatorMode,
    far_input: Option<SignalStrength>,
) -> SignalStrength {
    let mut input_power = node.default_inputs.power();
    let side_power = node.side_inputs.power();
    if let Some(far_input) = far_input
        && input_power < SignalStrength::Fifteen
    {
        input_power = far_input;
    }
    let difference = input_power.get().wrapping_sub(side_power.get());
    if difference <= SignalStrength::Fifteen.get() {
        match mode {
            ComparatorMode::Compare => input_power,
            ComparatorMode::Subtract => SignalStrength::try_from(difference).unwrap(),
        }
    } else {
        SignalStrength::Zero
    }
}
