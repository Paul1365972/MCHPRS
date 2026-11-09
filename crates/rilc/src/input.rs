use crate::numbers::counted;
use crate::Error;
use anyhow::Context;
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::BlockPos;
use mchprs_redstone::interpreter::Interpreter;
use mchprs_ril::ast;
use mchprs_save_data::plot_data::PlotData;
use mchprs_simulation::backend::Source;
use std::ffi::OsStr;
use std::fs;
use std::path::Path;

pub type World = Interpreter<BlockVolume>;

pub enum Loaded {
    World(World),
    Circuit(ast::Circuit),
}

pub fn source(world: &World) -> Source<'_> {
    Source {
        ticks: world.tick_entries(),
        ..Source::new(world, world.blocks().region())
    }
}

pub fn read_world(path: &Path) -> Result<World, Error> {
    let extension = path.extension().and_then(OsStr::to_str);
    if matches!(extension, Some("schem" | "schematic")) {
        let bytes = fs::read(path).with_context(|| format!("couldn't read {}", path.display()))?;
        let schematic = mchprs_schematic::read(&bytes)
            .with_context(|| format!("couldn't read {}", path.display()))?;
        for (name, count) in &schematic.unknown_blocks {
            eprintln!(
                "warning: {}: replaced {} of unknown or invalid state {name} with air",
                path.display(),
                counted(*count as u64, "block")
            );
        }
        let origin = schematic.volume.region().min();
        let volume = schematic.volume.translated(BlockPos::zero() - origin);
        return Ok(Interpreter::new(volume, Vec::new()));
    }
    let plot = PlotData::load_from_file(path)
        .with_context(|| format!("couldn't read {} as a plot", path.display()))?;
    let (volume, ticks) = plot.into_volume();
    Ok(Interpreter::new(volume, ticks))
}
