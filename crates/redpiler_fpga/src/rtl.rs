use std::path::{Path, PathBuf};
use std::{fs, io};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SourceFile<'a> {
    pub path: &'static str,
    pub text: &'a str,
}

macro_rules! source {
    ($path:literal) => {
        $crate::rtl::SourceFile {
            path: $path,
            text: include_str!(concat!("../rtl/", $path)),
        }
    };
}

const PACKAGE: SourceFile = source!("pkg.sv");
const CIRCUIT_PACKAGE: &str = "circuit_pkg.sv";
const CIRCUIT: &str = "circuit.sv";
const CIRCUIT_PORTS: SourceFile = source!("circuit_ports.svh");
const DESIGN: &[SourceFile] = &[
    source!("sequencer.sv"),
    source!("firing_copy.sv"),
    source!("nodes/timer.sv"),
    source!("nodes/lever.sv"),
    source!("nodes/strength_input.sv"),
    source!("nodes/button.sv"),
    source!("nodes/plate.sv"),
    source!("nodes/torch.sv"),
    source!("nodes/repeater.sv"),
    source!("nodes/repeater_prioritized.sv"),
    source!("nodes/comparator.sv"),
    source!("nodes/lamp.sv"),
    source!("nodes/trapdoor.sv"),
    source!("nodes/redstone_wire.sv"),
    source!("nodes/note_block.sv"),
    source!("controller.sv"),
    source!("handshake.sv"),
    source!("link/crc16.sv"),
    source!("link/frame_rx.sv"),
    source!("link/frame_tx.sv"),
];
pub const SIMULATION: [SourceFile; 2] = [source!("sim/top.sv"), source!("sim/main.cpp")];

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CircuitSource {
    pub package: String,
    pub module: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Sources {
    pub directory: PathBuf,
    /// In the order in which a tool must read them: a package before the modules that name it.
    pub files: Vec<&'static str>,
}

impl Sources {
    pub fn write<'a>(
        directory: &Path,
        circuit: &'a CircuitSource,
        top: impl IntoIterator<Item = SourceFile<'a>>,
    ) -> io::Result<Sources> {
        let generated = [
            (CIRCUIT_PACKAGE, &circuit.package),
            (CIRCUIT, &circuit.module),
        ];
        let generated = generated.map(|(path, text)| SourceFile { path, text });
        let design = [PACKAGE]
            .into_iter()
            .chain(generated)
            .chain(DESIGN.iter().copied());
        let read: Vec<_> = design.chain(top).collect();
        write(directory, read.iter().copied().chain([CIRCUIT_PORTS]))?;
        Ok(Sources {
            directory: directory.to_owned(),
            files: read.into_iter().map(|source| source.path).collect(),
        })
    }
}

pub fn write<'a>(
    directory: &Path,
    sources: impl IntoIterator<Item = SourceFile<'a>>,
) -> io::Result<()> {
    for source in sources {
        let path = directory.join(source.path);
        fs::create_dir_all(path.parent().unwrap_or(directory))?;
        fs::write(path, source.text)?;
    }
    Ok(())
}
