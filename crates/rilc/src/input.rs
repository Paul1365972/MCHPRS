use crate::numbers::counted;
use crate::Error;
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::BlockPos;
use mchprs_redstone::interpreter::Interpreter;
use mchprs_ril::{ast, Module};
use mchprs_save_data::plot_data::PlotData;
use std::ffi::OsStr;
use std::fs;
use std::path::Path;

pub type World = Interpreter<BlockVolume>;

pub enum Input {
    World(World),
    Circuit(ast::Circuit),
}

impl Input {
    pub fn read(path: &Path, circuit: Option<&str>) -> Result<Input, Error> {
        let extension = path.extension().and_then(OsStr::to_str);
        if extension != Some("ril") {
            if circuit.is_some() {
                return Err(
                    format!("{} is no RIL file to pick a circuit from", path.display()).into(),
                );
            }
            return Ok(Input::World(read_world(path)?));
        }
        let source = fs::read_to_string(path)
            .map_err(|error| format!("couldn't read {}: {error}", path.display()))?;
        let module =
            Module::parse(&source).map_err(|error| format!("{}:{error}", path.display()))?;
        let circuit = match (circuit, module.circuits()) {
            (Some(name), _) => module
                .circuit(name)
                .ok_or_else(|| format!("{} has no circuit @{name}", path.display()))?,
            (None, [circuit]) => circuit,
            (None, []) => return Err(format!("{} holds no circuit", path.display()).into()),
            (None, circuits) => {
                let names: Vec<_> = circuits
                    .iter()
                    .map(|circuit| format!("@{}", circuit.name))
                    .collect();
                return Err(format!(
                    "{} holds several circuits, so name one of {}",
                    path.display(),
                    names.join(", ")
                )
                .into());
            }
        };
        Ok(Input::Circuit(circuit.clone()))
    }
}

pub fn read_world(path: &Path) -> Result<World, Error> {
    let extension = path.extension().and_then(OsStr::to_str);
    if matches!(extension, Some("schem" | "schematic")) {
        let bytes =
            fs::read(path).map_err(|error| format!("couldn't read {}: {error}", path.display()))?;
        let schematic = mchprs_schematic::read(&bytes)
            .map_err(|error| format!("couldn't read {}: {error}", path.display()))?;
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
        .map_err(|error| format!("couldn't read {} as a plot: {error}", path.display()))?;
    let (volume, ticks) = plot.into_volume();
    Ok(Interpreter::new(volume, ticks))
}
