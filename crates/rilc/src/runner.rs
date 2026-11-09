use crate::input::{Loaded, World};
use crate::scenario::{Action, Scenario, Target};
use crate::setup::Setup;
use crate::Error;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::signal_strength::UncappedSignalStrength;
use mchprs_blocks::BlockPos;
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::graph::{Graph, NodeIdx, NodeKind, NodeState};
use mchprs_redpiler::nodes::{Changes, Nodes};
use mchprs_redstone::input::Input;
use mchprs_redstone::sound::Sound;
use mchprs_ril::{ast, component};
use mchprs_simulation::component::strength_at;
use mchprs_simulation::ports::Role;
use mchprs_simulation::simulated_world::SimulatedWorld;
use mchprs_simulation::{Simulation, View};
use rustc_hash::FxHashSet;
use std::mem;

pub type Shown = Option<(Component, ShownState)>;

#[derive(Clone, Copy)]
pub enum Component {
    Block(Block),
    Node(NodeKind, NodeState),
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct ShownState {
    pub power: UncappedSignalStrength,
    pub repeater_locked: bool,
}

/// Slots index the targets, which are sorted
pub trait Run {
    fn targets(&self) -> &[Target];
    fn component(&self, slot: usize) -> usize;
    fn shown(&mut self, slot: usize) -> Result<Shown, Error>;
    fn port(&self, slot: usize) -> Option<Role>;
    fn drive(&mut self, slot: usize, input: Input) -> Result<(), Error>;
    fn run(&mut self, ticks: u64) -> Result<(), Error>;
    fn take_sounds(&mut self) -> Result<Vec<(usize, Sound)>, Error>;
}

pub fn start(input: &Loaded, setup: &Setup) -> Result<Box<dyn Run>, Error> {
    Ok(match input {
        Loaded::World(world) => Box::new(WorldRun::new(world, setup)?),
        Loaded::Circuit(circuit) => Box::new(CircuitRun::new(labelled_graph(circuit), setup)?),
    })
}

pub fn labelled_graph(circuit: &ast::Circuit) -> Graph {
    let mut graph = circuit.graph();
    let nodes: Vec<NodeIdx> = graph.node_indices().collect();
    for (node, component) in nodes.into_iter().zip(&circuit.components) {
        graph[node].names = component.labels.clone();
    }
    graph
}

pub fn input_slots(run: &mut dyn Run, scenario: &Scenario) -> Result<Vec<Option<usize>>, Error> {
    scenario
        .steps
        .iter()
        .enumerate()
        .map(|(index, step)| {
            let Some(action) = &step.action else {
                return Ok(None);
            };
            let slot =
                input_slot(run, action).map_err(|problem| scenario.step_error(index, problem))?;
            Ok(Some(slot))
        })
        .collect()
}

pub fn describe(shown: Shown) -> String {
    match shown {
        Some((Component::Node(kind, state), _)) => component(kind, state).to_string(),
        Some((Component::Block(block), state)) => format!("{block} power {}", state.power.get()),
        None => "no component".to_owned(),
    }
}

fn input_slot(run: &mut dyn Run, action: &Action) -> Result<usize, String> {
    let target = &action.at;
    let Ok(slot) = run.targets().binary_search(target) else {
        return Err(match target {
            Target::Position(_) => format!("no component at {target}"),
            Target::Name(_) => format!("no node {target}"),
        });
    };
    let shown = run.shown(slot).map_err(|error| error.to_string())?;
    if run.port(slot) != Some(Role::Input) {
        return Err(format!(
            "{target} is {}, which is not an input",
            describe(shown)
        ));
    }
    Ok(slot)
}

struct WorldRun {
    simulated_world: SimulatedWorld,
    targets: Vec<Target>,
}

impl WorldRun {
    fn new(world: &World, setup: &Setup) -> Result<WorldRun, Error> {
        let region = world.blocks().region();
        let simulated_world = setup.simulate(world)?;
        let mut targets = Vec::new();
        for pos in world.occupied_positions(region) {
            if strength_at(world, pos).is_some() && simulated_world.is_current(pos) {
                targets.push(Target::from(pos));
            }
        }
        targets.sort();
        Ok(WorldRun {
            simulated_world,
            targets,
        })
    }

    fn pos(&self, slot: usize) -> BlockPos {
        let Target::Position([x, y, z]) = self.targets[slot] else {
            unreachable!("a world's targets are positions");
        };
        BlockPos::new(x, y, z)
    }
}

impl Run for WorldRun {
    fn targets(&self) -> &[Target] {
        &self.targets
    }

    fn component(&self, slot: usize) -> usize {
        slot
    }

    fn shown(&mut self, slot: usize) -> Result<Shown, Error> {
        let pos = self.pos(slot);
        let block = self.simulated_world.block(pos);
        Ok(strength_at(&self.simulated_world, pos).map(|power| {
            let repeater_locked = matches!(block, Block::Repeater(repeater) if repeater.locked);
            (
                Component::Block(block),
                ShownState {
                    power,
                    repeater_locked,
                },
            )
        }))
    }

    fn port(&self, slot: usize) -> Option<Role> {
        self.simulated_world.layout().at(self.pos(slot))?.port
    }

    fn drive(&mut self, slot: usize, input: Input) -> Result<(), Error> {
        Ok(self.simulated_world.drive(self.pos(slot), input)?)
    }

    fn run(&mut self, ticks: u64) -> Result<(), Error> {
        self.simulated_world.run(ticks)?;
        Ok(())
    }

    fn take_sounds(&mut self) -> Result<Vec<(usize, Sound)>, Error> {
        let sounds = self.simulated_world.take_sounds().into_iter();
        let slots = sounds.filter_map(|(pos, sound)| {
            let slot = self.targets.binary_search(&Target::from(pos)).ok()?;
            Some((slot, sound))
        });
        Ok(slots.collect())
    }
}

struct CircuitRun {
    circuit: Circuit,
    simulation: Box<dyn Simulation<Nodes>>,
    targets: Vec<Target>,
    nodes: Vec<NodeId>,
    states: Vec<NodeState>,
    played_note_blocks: Vec<NodeId>,
}

impl CircuitRun {
    fn new(graph: Graph, setup: &Setup) -> Result<CircuitRun, Error> {
        let (circuit, simulation) = setup.build(graph)?;
        let mut slots: Vec<(Target, NodeId)> = Vec::new();
        for id in circuit.ids().filter(|&id| circuit.node(id).observed) {
            let node = circuit.node(id);
            let names = node.names.iter().map(|name| Target::Name(name.clone()));
            let positions = node.blocks.iter().map(|&(pos, _)| Target::from(pos));
            slots.extend(names.chain(positions).map(|target| (target, id)));
        }
        slots.sort();
        let (targets, nodes) = slots.into_iter().unzip();
        Ok(CircuitRun {
            states: circuit.initial().states,
            circuit,
            simulation,
            targets,
            nodes,
            played_note_blocks: Vec::new(),
        })
    }

    fn show(&mut self) -> Result<(), Error> {
        let mut changes = Changes::default();
        self.simulation.show(View::Observed, &mut changes)?;
        for (node, state) in changes.states {
            self.states[node.index()] = state;
        }
        self.played_note_blocks
            .append(&mut changes.played_note_blocks);
        Ok(())
    }
}

impl Run for CircuitRun {
    fn targets(&self) -> &[Target] {
        &self.targets
    }

    fn component(&self, slot: usize) -> usize {
        self.nodes[slot].index()
    }

    fn shown(&mut self, slot: usize) -> Result<Shown, Error> {
        let node = self.nodes[slot];
        let kind = self.circuit.node(node).kind;
        let state = self.states[node.index()];
        Ok(Some((
            Component::Node(kind, state),
            ShownState {
                power: state.power.into(),
                repeater_locked: state.repeater_locked,
            },
        )))
    }

    fn port(&self, slot: usize) -> Option<Role> {
        self.circuit.node(self.nodes[slot]).port
    }

    fn drive(&mut self, slot: usize, input: Input) -> Result<(), Error> {
        self.simulation.drive(self.nodes[slot], input)?;
        self.show()
    }

    fn run(&mut self, ticks: u64) -> Result<(), Error> {
        self.simulation.run(ticks)?;
        self.show()
    }

    fn take_sounds(&mut self) -> Result<Vec<(usize, Sound)>, Error> {
        if self.played_note_blocks.is_empty() {
            return Ok(Vec::new());
        }
        let played: FxHashSet<NodeId> = mem::take(&mut self.played_note_blocks)
            .into_iter()
            .collect();
        let mut sounds = Vec::new();
        for (slot, node) in self.nodes.iter().enumerate() {
            if let NodeKind::NoteBlock { instrument, note } = self.circuit.node(*node).kind
                && played.contains(node)
            {
                sounds.push((slot, Sound::note(instrument, note)));
            }
        }
        Ok(sounds)
    }
}
