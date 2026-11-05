use super::{BoxError, Configured, Source};
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::blocks::{Block, Instrument};
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::BlockPos;
use mchprs_redpiler::block_map::PlayedNote;
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::engine::{Input, Snapshot};
use mchprs_redpiler::graph::{DriveKind, PendingTick};
use mchprs_redpiler::ports::Role;
use mchprs_redpiler::simulation::Simulation;
use rustc_hash::FxHashMap;
use std::mem;
use std::num::NonZeroU64;

const EXPLAINED_DIFFERENCES: usize = 12;

pub fn diff(
    source: &Source,
    backends: [Configured; 2],
    ticks: u64,
    drive_interval: Option<NonZeroU64>,
) -> Result<bool, BoxError> {
    let [first, second] = backends;
    let mut subjects = [
        Subject::start(first, source)?,
        Subject::start(second, source)?,
    ];
    let names = subjects.each_ref().map(|subject| subject.name.clone());
    let unshown = unshown_ports(&subjects);
    if !unshown.is_empty() {
        println!("{} and {} differ in their ports", names[0], names[1]);
        for (name, position) in unshown {
            println!("{name} doesn't show the port at {position}");
        }
        return Ok(false);
    }
    let same_circuit = subjects[0].circuit() == subjects[1].circuit();
    let inputs = input_ports(subjects[0].circuit(), same_circuit);
    let mut driven = 0;
    let mut last_drive = None;
    for tick in 1..=ticks {
        if let Some(interval) = drive_interval
            && (tick - 1) % interval == 0
            && !inputs.is_empty()
        {
            let port = inputs[driven % inputs.len()];
            let input = next_input(&subjects[0], port)?;
            for subject in &mut subjects {
                subject.drive(port, input)?;
            }
            driven += 1;
            last_drive = Some((tick, port, input));
        }
        let ticked = [subjects[0].tick()?, subjects[1].tick()?];
        let differences = differences(&subjects, &ticked, same_circuit);
        if !differences.is_empty() {
            println!("{} and {} differ after tick {tick}", names[0], names[1]);
            if let Some((tick, port, input)) = last_drive {
                let circuit = subjects[0].circuit();
                println!(
                    "last input: {input:?} on {} before tick {tick}",
                    describe_node(circuit, port.node(circuit))
                );
            }
            explain(&subjects, &ticked, &differences);
            return Ok(false);
        }
    }
    let compared = if same_circuit {
        "node by node".to_owned()
    } else {
        format!("at the {} positions both show", shown_by_both(&subjects))
    };
    println!(
        "{} and {} match for {ticks} ticks ({driven} inputs driven, compared {compared})",
        names[0], names[1]
    );
    Ok(true)
}

fn unshown_ports(subjects: &[Subject; 2]) -> Vec<(&str, BlockPos)> {
    let mut ports: Vec<BlockPos> = subjects
        .iter()
        .flat_map(|subject| {
            let circuit = subject.circuit();
            circuit
                .ports()
                .flat_map(move |(node, _)| circuit.node(node).blocks.iter())
                .map(|&(position, _)| position)
        })
        .collect();
    ports.sort_by_key(|&position| position_key(position));
    ports.dedup();
    ports
        .into_iter()
        .flat_map(|position| {
            subjects
                .iter()
                .filter(move |subject| !subject.shown.contains_key(&position))
                .map(move |subject| (subject.name.as_str(), position))
        })
        .collect()
}

fn shown_by_both(subjects: &[Subject; 2]) -> usize {
    subjects[0]
        .shown
        .keys()
        .filter(|position| subjects[1].shown.contains_key(position))
        .count()
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct ShownBlock {
    block: Block,
    comparator_strength: Option<SignalStrength>,
}

struct Subject {
    name: String,
    simulation: Simulation,
    shown: FxHashMap<BlockPos, ShownBlock>,
    state: Snapshot,
}

struct Ticked {
    before: Snapshot,
    changed: Vec<BlockPos>,
    notes: Vec<PlayedNote>,
}

impl Subject {
    fn start(backend: Configured, source: &Source) -> Result<Subject, BoxError> {
        let name = backend.setup.to_string();
        let (circuit, engine) = backend.start(source)?;
        let mut simulation = Simulation::new(circuit.into_owned(), engine);
        let circuit = simulation.circuit();
        let shown = circuit
            .ids()
            .map(|id| circuit.node(id))
            .filter(|node| node.observed)
            .flat_map(|node| {
                node.blocks.iter().map(|&(position, id)| {
                    let block = Block::from_id(id);
                    let comparator_strength =
                        matches!(block, Block::Comparator(_)).then_some(node.state.power);
                    let shown = ShownBlock {
                        block,
                        comparator_strength,
                    };
                    (position, shown)
                })
            })
            .collect();
        Ok(Subject {
            name,
            state: simulation.snapshot()?,
            simulation,
            shown,
        })
    }

    fn circuit(&self) -> &Circuit {
        self.simulation.circuit()
    }

    fn drive(&mut self, port: InputPort, input: Input) -> Result<(), BoxError> {
        match port {
            InputPort::Node(node) => self.simulation.drive_node(node, input),
            InputPort::Position(position) => self.simulation.drive(position, input),
        }
        .map_err(|error| format!("{}: {error}", self.name))?;
        self.state = self.simulation.snapshot()?;
        Ok(())
    }

    fn tick(&mut self) -> Result<Ticked, BoxError> {
        self.simulation.run(1)?;
        let changes = self.simulation.sync()?;
        for &(position, block) in &changes.blocks {
            self.shown_at(position).block = block;
        }
        for (position, block_entity) in changes.block_entities {
            if let BlockEntity::Comparator { output_strength } = block_entity {
                self.shown_at(position).comparator_strength = Some(output_strength);
            }
        }
        let mut notes = changes.notes;
        notes.sort_by_key(|note| position_key(note.position));
        Ok(Ticked {
            before: mem::replace(&mut self.state, self.simulation.snapshot()?),
            changed: changes
                .blocks
                .iter()
                .map(|&(position, _)| position)
                .collect(),
            notes,
        })
    }

    fn shown_at(&mut self, position: BlockPos) -> &mut ShownBlock {
        self.shown
            .get_mut(&position)
            .expect("a simulation only changes the blocks of observed nodes")
    }
}

#[derive(Debug, Clone, Copy)]
enum InputPort {
    Node(NodeId),
    Position(BlockPos),
}

impl InputPort {
    fn node(self, circuit: &Circuit) -> NodeId {
        match self {
            InputPort::Node(node) => node,
            InputPort::Position(position) => circuit
                .node_at(position)
                .expect("inputs come from this circuit"),
        }
    }
}

fn input_ports(circuit: &Circuit, same_circuit: bool) -> Vec<InputPort> {
    let inputs = circuit
        .ports()
        .filter(|&(_, role)| role == Role::Input)
        .map(|(node, _)| node);
    if same_circuit {
        inputs.map(InputPort::Node).collect()
    } else {
        inputs
            .filter_map(|node| Some(InputPort::Position(circuit.node(node).blocks.first()?.0)))
            .collect()
    }
}

fn next_input(subject: &Subject, port: InputPort) -> Result<Input, BoxError> {
    let circuit = subject.circuit();
    let node = port.node(circuit);
    Ok(match circuit.node(node).kind.drive_kind() {
        Some(DriveKind::OnOff) => Input::Toggle,
        Some(DriveKind::Press) => Input::Press,
        Some(DriveKind::Strength) if subject.state.states[node.index()].power.is_zero() => {
            Input::Set(SignalStrength::Fifteen)
        }
        Some(DriveKind::Strength) => Input::Set(SignalStrength::Zero),
        None => return Err(format!("{} can't be driven", describe_node(circuit, node)).into()),
    })
}

enum Difference {
    Node(NodeId),
    Position(BlockPos),
}

fn differences(
    subjects: &[Subject; 2],
    ticked: &[Ticked; 2],
    same_circuit: bool,
) -> Vec<Difference> {
    let mut differences = Vec::new();
    let mut positions = differing_notes(subjects, ticked);
    if same_circuit {
        differences.extend(differing_nodes(subjects).map(Difference::Node));
    } else {
        positions.extend(differing_blocks(subjects, ticked));
    }
    positions.sort_by_key(|&position| position_key(position));
    positions.dedup();
    differences.extend(positions.into_iter().map(Difference::Position));
    differences
}

fn differing_nodes(subjects: &[Subject; 2]) -> impl Iterator<Item = NodeId> + '_ {
    let [first, second] = subjects.each_ref().map(|subject| &subject.state);
    let pending = [first, second].map(pending_by_node);
    subjects[0].circuit().ids().filter(move |node| {
        let index = node.index();
        first.states[index] != second.states[index] || pending[0][index] != pending[1][index]
    })
}

fn differing_blocks<'a>(
    subjects: &'a [Subject; 2],
    ticked: &'a [Ticked; 2],
) -> impl Iterator<Item = BlockPos> + 'a {
    let changed = ticked.iter().flat_map(|ticked| &ticked.changed).copied();
    changed.filter(|position| {
        match subjects
            .each_ref()
            .map(|subject| subject.shown.get(position))
        {
            [Some(first), Some(second)] => first != second,
            _ => false,
        }
    })
}

fn differing_notes(subjects: &[Subject; 2], ticked: &[Ticked; 2]) -> Vec<BlockPos> {
    let played_at = ticked.iter().flat_map(|ticked| &ticked.notes);
    played_at
        .map(|note| note.position)
        .filter(|position| {
            subjects
                .iter()
                .all(|subject| subject.shown.contains_key(position))
        })
        .filter(|&position| !played(&ticked[0], position).eq(played(&ticked[1], position)))
        .collect()
}

fn played(ticked: &Ticked, position: BlockPos) -> impl Iterator<Item = (Instrument, u8, u32)> + '_ {
    ticked
        .notes
        .iter()
        .filter(move |note| note.position == position)
        .map(|note| (note.instrument, note.note, note.count))
}

fn position_key(position: BlockPos) -> (i32, i32, i32) {
    (position.x, position.y, position.z)
}

fn explain(subjects: &[Subject; 2], ticked: &[Ticked; 2], differences: &[Difference]) {
    match differences.len() {
        1 => println!("1 difference"),
        count => println!("{count} differences"),
    }
    for difference in differences.iter().take(EXPLAINED_DIFFERENCES) {
        match *difference {
            Difference::Node(node) => {
                let circuit = subjects[0].circuit();
                println!("{}", describe_node(circuit, node));
                for subject in subjects {
                    println!(
                        "    {}: {}",
                        subject.name,
                        describe_state(&subject.state, node)
                    );
                }
                describe_inputs(circuit, node, &ticked[0].before, "    ");
            }
            Difference::Position(position) => {
                println!("at {position}");
                for (subject, ticked) in subjects.iter().zip(ticked) {
                    let shown = subject.shown[&position];
                    print!("    {} shows {:?}", subject.name, shown.block);
                    if let Some(strength) = shown.comparator_strength {
                        print!(" with strength {}", strength.get());
                    }
                    let notes: Vec<_> = played(ticked, position).collect();
                    if !notes.is_empty() {
                        print!(" and played {notes:?}");
                    }
                    println!();
                    let circuit = subject.circuit();
                    let node = circuit
                        .node_at(position)
                        .expect("observed positions have a node");
                    println!(
                        "        {}: {}",
                        describe_node(circuit, node),
                        describe_state(&subject.state, node)
                    );
                    describe_inputs(circuit, node, &ticked.before, "        ");
                }
            }
        }
    }
}

fn describe_inputs(circuit: &Circuit, node: NodeId, before: &Snapshot, indent: &str) {
    for link in circuit.inputs(node) {
        println!(
            "{indent}{:?} input with weight {} from {}, before the tick: {}",
            link.kind,
            link.weight,
            describe_node(circuit, link.source),
            describe_state(before, link.source)
        );
    }
}

fn pending_by_node(snapshot: &Snapshot) -> Vec<Option<(PendingTick, usize)>> {
    let mut pending = vec![None; snapshot.states.len()];
    let mut queued: FxHashMap<PendingTick, usize> = FxHashMap::default();
    for &(node, tick) in &snapshot.pending_ticks {
        let ahead = queued.entry(tick).or_default();
        pending[node.index()] = Some((tick, *ahead));
        *ahead += 1;
    }
    pending
}

fn describe_node(circuit: &Circuit, node: NodeId) -> String {
    let kind = &circuit.node(node).kind;
    match circuit.node(node).blocks.first() {
        Some((position, _)) => format!("node {} {kind:?} at {position}", node.index()),
        None => format!("node {} {kind:?}", node.index()),
    }
}

fn describe_state(snapshot: &Snapshot, node: NodeId) -> String {
    let state = snapshot.states[node.index()];
    let mut description = format!("power {}", state.power.get());
    if state.repeater_locked {
        description.push_str(", locked");
    }
    if let Some((tick, ahead)) = pending_by_node(snapshot)[node.index()] {
        description.push_str(&format!(
            ", pending in {} at {:?} with {ahead} ahead",
            tick.delay, tick.priority
        ));
    }
    description
}
