use crate::input::{read_world, settle};
use crate::{Error, Outcome};
use anyhow::Context;
use std::fs;
use std::path::Path;

pub fn command(input: &Path, output: &Path, settle_limit: u64) -> Outcome {
    let mut world = read_world(input)?;
    let ticks = settle(&mut world, settle_limit)?;
    let schematic = mchprs_schematic::write(world.blocks()).map_err(Error::from)?;
    fs::write(output, schematic).with_context(|| format!("couldn't write {}", output.display()))?;
    println!("settled after {ticks} ticks and wrote {}", output.display());
    Ok(true)
}
