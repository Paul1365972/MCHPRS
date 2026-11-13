use crate::numbers::counted;
use crate::Error;
use anyhow::{anyhow, Context};
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::BlockPos;
use mchprs_redstone::interpreter::Interpreter;
use mchprs_ril::{ast, Module};
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

impl Loaded {
    pub fn read(path: &Path, circuit: Option<&str>) -> Result<Loaded, Error> {
        let extension = path.extension().and_then(OsStr::to_str);
        if extension != Some("ril") {
            if circuit.is_some() {
                return Err(anyhow!(
                    "{} is no RIL file to pick a circuit from",
                    path.display()
                ));
            }
            return Ok(Loaded::World(read_world(path)?));
        }
        let source = fs::read_to_string(path)
            .with_context(|| format!("couldn't read {}", path.display()))?;
        let module =
            Module::parse(&source).map_err(|error| anyhow!("{}:{error}", path.display()))?;
        let circuit = match (circuit, module.circuits()) {
            (Some(name), _) => module
                .circuit(name)
                .ok_or_else(|| anyhow!("{} has no circuit @{name}", path.display()))?,
            (None, [circuit]) => circuit,
            (None, []) => return Err(anyhow!("{} holds no circuit", path.display())),
            (None, circuits) => {
                let names: Vec<_> = circuits
                    .iter()
                    .map(|circuit| format!("@{}", circuit.name))
                    .collect();
                return Err(anyhow!(
                    "{} holds several circuits, so name one of {}",
                    path.display(),
                    names.join(", ")
                ));
            }
        };
        Ok(Loaded::Circuit(circuit.clone()))
    }
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

pub fn settle(world: &mut World, limit: u64) -> Result<u64, Error> {
    let region = world.blocks().region();
    mchprs_redstone::settle(world, region, limit)
        .ok_or_else(|| anyhow!("the world hasn't settled within {}", counted(limit, "tick")))
}
