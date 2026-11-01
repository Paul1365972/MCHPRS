mod node;
mod tick;
mod update;

use crate::circuit::{self, Circuit};
use crate::engine::{Changes, Engine, EngineError, Input, Snapshot};
use crate::graph::{self, LinkKind, PendingTick};
use mchprs_blocks::blocks::ComparatorMode;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_redstone::tick_queue::TickPriority;
use node::{ForwardLink, ForwardLinks, Node, NodeId, NodeInput, NodeKind, Nodes};
use std::mem;

#[derive(Default, Clone)]
struct Queues([Vec<NodeId>; TickScheduler::NUM_PRIORITIES]);

impl Queues {
    #[inline(always)]
    fn drain_each<F: FnMut(NodeId)>(&mut self, mut f: F) {
        for q in self.0.iter_mut() {
            for n in q.iter() {
                f(*n);
            }
            q.clear();
        }
    }
}

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

#[derive(Default)]
struct TickScheduler {
    queues_deque: [Queues; Self::NUM_QUEUES],
    pos: usize,
}

impl TickScheduler {
    const NUM_PRIORITIES: usize = 4;
    const NUM_QUEUES: usize = 16;

    fn schedule_tick(&mut self, node: NodeId, delay: usize, priority: TickPriority) {
        self.queues_deque[(self.pos + delay) % Self::NUM_QUEUES].0[priority as usize].push(node);
    }

    fn queues_this_tick(&mut self) -> Queues {
        self.pos = (self.pos + 1) % Self::NUM_QUEUES;
        mem::take(&mut self.queues_deque[self.pos])
    }

    fn end_tick(&mut self, queues: Queues) {
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

    fn has_pending_ticks(&self) -> bool {
        self.queues_deque
            .iter()
            .any(|queues| queues.0.iter().any(|queue| !queue.is_empty()))
    }

    fn pending_ticks(&self) -> impl Iterator<Item = (NodeId, PendingTick)> + '_ {
        (1..=Self::NUM_QUEUES).flat_map(move |delay| {
            let queues = &self.queues_deque[(self.pos + delay) % Self::NUM_QUEUES];
            queues
                .0
                .iter()
                .zip(Self::priorities())
                .flat_map(move |(queue, priority)| {
                    queue.iter().map(move |&node| {
                        let delay = delay as u32;
                        (node, PendingTick { delay, priority })
                    })
                })
        })
    }

    fn clear(&mut self) {
        for queues in &mut self.queues_deque {
            for queue in &mut queues.0 {
                queue.clear();
            }
        }
    }
}

pub struct DirectEngine {
    nodes: Nodes,
    forward_links: ForwardLinks,
    scheduler: TickScheduler,
    notes: NoteActivations,
    initial: Snapshot,
}

impl DirectEngine {
    const MAX_LINKS_PER_KIND: usize = u8::MAX as usize;

    pub fn new(circuit: &Circuit) -> Result<DirectEngine, EngineError> {
        for id in circuit.ids() {
            for kind in [LinkKind::Default, LinkKind::Side] {
                let count = circuit
                    .inputs(id)
                    .iter()
                    .filter(|link| link.kind == kind)
                    .count();
                if count > Self::MAX_LINKS_PER_KIND {
                    return Err(format!(
                        "node {} has {count} {kind:?} inputs (at most {} supported)",
                        id.index(),
                        Self::MAX_LINKS_PER_KIND
                    )
                    .into());
                }
            }
        }
        let initial = circuit.initial();
        check_delays(&initial.pending_ticks)?;
        let mut forward_links = ForwardLinks::default();
        let nodes = circuit
            .ids()
            .map(|id| build_node(circuit, id, &mut forward_links))
            .collect();
        let mut engine = DirectEngine {
            nodes: Nodes::new(nodes),
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
            let node = self.nodes.get(node.index());
            self.scheduler
                .schedule_tick(node, tick.delay as usize, tick.priority);
            self.nodes[node].pending_tick = true;
        }
    }

    #[inline(always)]
    fn tick(&mut self) {
        let mut queues = self.scheduler.queues_this_tick();
        // Like vanilla, a Lamp may schedule a new tick before its tick in this tick has run
        for queue in &queues.0 {
            for &node_id in queue {
                let node = &mut self.nodes[node_id];
                if matches!(node.kind, NodeKind::Lamp) {
                    node.pending_tick = false;
                }
            }
        }

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
    fn drive(&mut self, node: circuit::NodeId, input: Input) -> Result<(), EngineError> {
        let node_id = self.nodes.get(node.index());
        match input {
            Input::Press => {
                let node = &mut self.nodes[node_id];
                if !node.is_powered() {
                    if !node.pending_tick {
                        schedule_tick(&mut self.scheduler, node_id, node, 10, TickPriority::Normal);
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
        for (index, node) in self.nodes.inner_mut().iter_mut().enumerate() {
            if node.changed && node.visible {
                node.changed = false;
                changes
                    .states
                    .push((circuit::NodeId::new(index), node.state()));
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
            node.pending_tick = false;
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

fn check_delays(pending_ticks: &[(circuit::NodeId, PendingTick)]) -> Result<(), EngineError> {
    for &(node, tick) in pending_ticks {
        if tick.delay as usize > TickScheduler::NUM_QUEUES {
            return Err(format!(
                "pending tick of node {} is {} ticks away (at most {} supported)",
                node.index(),
                tick.delay,
                TickScheduler::NUM_QUEUES
            )
            .into());
        }
    }
    Ok(())
}

fn build_node(circuit: &Circuit, id: circuit::NodeId, forward_links: &mut ForwardLinks) -> Node {
    let node = circuit.node(id);
    let fwd_link_range = forward_links.extend(
        circuit
            .outputs(id)
            .iter()
            .filter(|link| link.weight < 15)
            .map(|link| {
                let target = NodeId::new(link.target.index(), circuit.len());
                ForwardLink::new(target, link.kind == LinkKind::Side, link.weight)
            }),
    );

    let kind = match node.kind {
        graph::NodeKind::Repeater { delay, prioritized } => {
            NodeKind::Repeater { delay, prioritized }
        }
        graph::NodeKind::Torch => NodeKind::Torch,
        graph::NodeKind::Comparator {
            mode,
            far_input,
            prioritized,
        } => NodeKind::Comparator {
            mode,
            far_input,
            prioritized,
        },
        graph::NodeKind::Lamp => NodeKind::Lamp,
        graph::NodeKind::Button => NodeKind::Button,
        graph::NodeKind::Lever => NodeKind::Lever,
        graph::NodeKind::PressurePlate => NodeKind::PressurePlate,
        graph::NodeKind::Trapdoor => NodeKind::Trapdoor,
        graph::NodeKind::Wire => NodeKind::Wire,
        graph::NodeKind::Constant => NodeKind::Constant,
        graph::NodeKind::NoteBlock { .. } => NodeKind::NoteBlock,
    };

    Node {
        kind,
        default_inputs: NodeInput::new(),
        side_inputs: NodeInput::new(),
        fwd_link_range,
        power: node.state.power,
        repeater_locked: node.state.repeater_locked,
        changed: false,
        pending_tick: false,
        visible: node.observed,
    }
}

fn schedule_tick(
    scheduler: &mut TickScheduler,
    node_id: NodeId,
    node: &mut Node,
    delay: usize,
    priority: TickPriority,
) {
    node.pending_tick = true;
    scheduler.schedule_tick(node_id, delay, priority);
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
