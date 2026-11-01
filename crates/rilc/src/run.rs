use crate::experiment::{boundaries, position, run_until, Loaded, Observation, Result};
use mchprs_blocks::BlockPos;
use mchprs_world::testing::TestWorld;
use mchprs_world::World;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::io::Write;
use std::path::Path;
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
    let mut checkpoints = Vec::new();
    let mut inputs = experiment.inputs.iter().peekable();
    let mut completed = 0;
    let started = Instant::now();
    for boundary in boundaries(experiment, observation.and_then(|observe| observe.every)) {
        run_until(engine.as_mut(), boundary - completed)?;
        completed = boundary;
        while let Some(input) = inputs.next_if(|input| input.tick == completed) {
            engine.input(compiled.node_at(*input)?, input.input())?;
        }
        compiled
            .block_map
            .translate(&engine.take_changes()?)
            .apply(&mut loaded.world);
        if let Some(observation) = observation {
            checkpoints.push(checkpoint(&loaded.world, observation, completed));
        }
        eprintln!("tick {completed}/{}", experiment.ticks);
    }
    let run_seconds = started.elapsed().as_secs_f64();

    let report = json!({
        "rilc": env!("GIT_HASH").trim(),
        "experiment": loaded.path,
        "plot_sha256": loaded.plot_sha256,
        "backend": backend,
        "options": experiment.options,
        "ticks": experiment.ticks,
        "nodes": compiled.netlist.len(),
        "compile_seconds": compiled.compile_seconds,
        "prepare_seconds": prepare_seconds,
        "stages": compiled.stages.borrow().as_slice(),
        "run_seconds": run_seconds,
        "ticks_per_second": experiment.ticks as f64 / run_seconds,
        "checkpoints": checkpoints,
    });
    serde_json::to_writer(&mut *output, &report)?;
    writeln!(output)?;
    Ok(())
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
