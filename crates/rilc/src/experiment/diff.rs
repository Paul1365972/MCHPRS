use super::*;

/// How many of the differing nodes a diff explains
const EXPLAINED_NODES: usize = 12;

pub fn diff(
    circuit: Circuit,
    engine_kinds: [EngineKind; 2],
    ticks: u64,
    drive_interval: Option<NonZeroU64>,
) -> Result<bool, Box<dyn Error + Send + Sync>> {
    let circuit = Arc::new(circuit);
    let mut engines = [
        engine_kinds[0].build(circuit.clone())?,
        engine_kinds[1].build(circuit.clone())?,
    ];
    let inputs: Vec<NodeId> = circuit
        .ports()
        .filter(|(_, role)| *role == Role::Input)
        .map(|(node, _)| node)
        .collect();
    let mut driven = 0;
    let mut last_drive = None;
    let mut before = engines[0].snapshot()?;
    for tick in 1..=ticks {
        if let Some(interval) = drive_interval
            && (tick - 1) % interval == 0
            && !inputs.is_empty()
        {
            let node = inputs[driven % inputs.len()];
            let input = match circuit.node(node).kind.drive() {
                Some(Drive::OnOff) => Input::Toggle,
                Some(Drive::Press) => Input::Press,
                Some(Drive::Strength) if before.states[node.index()].power.is_zero() => {
                    Input::Set(SignalStrength::Fifteen)
                }
                Some(Drive::Strength) => Input::Set(SignalStrength::Zero),
                None => {
                    return Err(format!("{} can't be driven", describe_node(&circuit, node)).into());
                }
            };
            circuit.check_input(node, input)?;
            for engine in &mut engines {
                engine.drive(node, input)?;
            }
            driven += 1;
            last_drive = Some((tick, node, input));
            before = engines[0].snapshot()?;
        }
        for engine in &mut engines {
            engine.run(1)?;
        }
        let after = [engines[0].snapshot()?, engines[1].snapshot()?];
        let played = [
            played_by_node(&mut engines[0], circuit.len())?,
            played_by_node(&mut engines[1], circuit.len())?,
        ];
        let same = after[0].states == after[1].states
            && pending_by_node(&after[0]) == pending_by_node(&after[1])
            && played[0] == played[1];
        if !same {
            println!(
                "{} and {} differ after tick {tick}",
                engine_kinds[0], engine_kinds[1]
            );
            if let Some((tick, node, input)) = last_drive {
                println!(
                    "last input: {input:?} on {} before tick {tick}",
                    describe_node(&circuit, node)
                );
            }
            explain(&circuit, engine_kinds, &before, &after, &played);
            return Ok(false);
        }
        let [left, _] = after;
        before = left;
    }
    println!(
        "{} and {} are the same for {ticks} ticks with {driven} inputs driven",
        engine_kinds[0], engine_kinds[1]
    );
    Ok(true)
}

fn explain(
    circuit: &Circuit,
    engine_kinds: [EngineKind; 2],
    before: &Snapshot,
    after: &[Snapshot; 2],
    played: &[Vec<u32>; 2],
) {
    let pending = after.each_ref().map(pending_by_node);
    let differing: Vec<NodeId> = circuit
        .ids()
        .filter(|node| {
            after[0].states[node.index()] != after[1].states[node.index()]
                || pending[0][node.index()] != pending[1][node.index()]
                || played[0][node.index()] != played[1][node.index()]
        })
        .collect();
    println!("{} nodes differ", differing.len());
    let before_pending = pending_by_node(before);
    for &node in differing.iter().take(EXPLAINED_NODES) {
        println!("{}", describe_node(circuit, node));
        for index in 0..2 {
            let mut state = describe_state(&after[index], &pending[index], node);
            match played[index][node.index()] {
                0 => {}
                1 => state.push_str(", played a note"),
                count => state.push_str(&format!(", played {count} notes")),
            }
            println!("    {}: {state}", engine_kinds[index]);
        }
        for link in circuit.inputs(node) {
            println!(
                "    {:?} input with weight {} from {}, before the tick: {}",
                link.kind,
                link.weight,
                describe_node(circuit, link.source),
                describe_state(before, &before_pending, link.source)
            );
        }
    }
}

fn played_by_node(engine: &mut impl Engine, nodes: usize) -> Result<Vec<u32>, EngineError> {
    let mut played = vec![0; nodes];
    for (node, count) in engine.changes()?.notes {
        played[node.index()] = count;
    }
    Ok(played)
}

fn pending_by_node(snapshot: &Snapshot) -> Vec<Option<PendingTick>> {
    let mut pending = vec![None; snapshot.states.len()];
    for &(node, tick) in &snapshot.pending_ticks {
        pending[node.index()] = Some(tick);
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

fn describe_state(snapshot: &Snapshot, pending: &[Option<PendingTick>], node: NodeId) -> String {
    let state = snapshot.states[node.index()];
    let mut description = format!("power {}", state.power.get());
    if state.repeater_locked {
        description.push_str(", locked");
    }
    if let Some(tick) = pending[node.index()] {
        description.push_str(&format!(
            ", pending in {} at {:?}",
            tick.delay, tick.priority
        ));
    }
    description
}
