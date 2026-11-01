use crate::experiment::{position, write_report, Loaded, Observation, Result, Schedule};
use mchprs_blocks::BlockPos;
use mchprs_world::testing::TestWorld;
use mchprs_world::World;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::io::Write;
use std::path::Path;
use std::slice;
use std::time::Instant;

pub fn run(path: &Path, backend: &str, output: &mut dyn Write) -> Result<()> {
    let mut loaded = Loaded::open(path)?;
    let mut compiled = loaded.compile();
    let prepare_started = Instant::now();
    let mut engine = compiled.engine(backend, loaded.options.reporting())?;
    let prepare_seconds = prepare_started.elapsed().as_secs_f64();
    loaded.world.to_be_ticked.clear();

    let experiment = &loaded.experiment;
    let observation = experiment.observe;
    let mut schedule = Schedule::new(experiment, observation.and_then(|observe| observe.every));
    let mut checkpoints = Vec::new();
    let started = Instant::now();
    while let Some(tick) = schedule.advance(&compiled, slice::from_mut(&mut engine))? {
        compiled
            .block_map
            .translate(&engine.take_changes()?)
            .apply(&mut loaded.world);
        if let Some(observation) = observation {
            checkpoints.push(checkpoint(&loaded.world, observation, tick));
        }
        eprintln!("tick {tick}/{}", experiment.ticks);
    }
    let run_seconds = started.elapsed().as_secs_f64();

    let details = json!({
        "backend": backend,
        "ticks": experiment.ticks,
        "compile_seconds": compiled.compile_seconds,
        "prepare_seconds": prepare_seconds,
        "run_seconds": run_seconds,
        "ticks_per_second": experiment.ticks as f64 / run_seconds,
        "checkpoints": checkpoints,
    });
    write_report(output, &loaded, &compiled, details)
}

fn checkpoint(world: &TestWorld, observation: Observation, tick: u64) -> Value {
    let (from, to) = (position(observation.from), position(observation.to));
    let (minimum, maximum) = (from.min(to), from.max(to));
    let mut blocks = Vec::new();
    for y in minimum.y..=maximum.y {
        for z in minimum.z..=maximum.z {
            for x in minimum.x..=maximum.x {
                blocks.push(world.get_block_raw(BlockPos::new(x, y, z)));
            }
        }
    }
    let mut hash = Sha256::new();
    for block in &blocks {
        hash.update(block.to_le_bytes());
    }
    json!({
        "tick": tick,
        "order": "y,z,x",
        "blocks": blocks,
        "sha256": format!("{:x}", hash.finalize()),
    })
}
