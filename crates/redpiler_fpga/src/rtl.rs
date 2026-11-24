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
pub(crate) use source;

const PACKAGE: SourceFile = source!("pkg.sv");
const CIRCUIT_PACKAGE: &str = "circuit_pkg.sv";
const NODES: &str = "nodes.sv";
const NODE_MODULES: &[SourceFile] = &[
    source!("nodes/timer.sv"),
    source!("nodes/binary_input.sv"),
    source!("nodes/strength_input.sv"),
    source!("nodes/button.sv"),
    source!("nodes/plate.sv"),
    source!("nodes/torch.sv"),
    source!("nodes/repeater.sv"),
    source!("nodes/repeater_prioritized.sv"),
    source!("nodes/repeater_pulse_free.sv"),
    source!("nodes/repeater_prioritized_pulse_free.sv"),
    source!("nodes/comparator.sv"),
    source!("nodes/comparator_binary.sv"),
    source!("nodes/lamp.sv"),
    source!("nodes/trapdoor.sv"),
    source!("nodes/redstone_wire.sv"),
    source!("nodes/note_block.sv"),
];
const DESIGN: &[SourceFile] = &[
    source!("circuit.sv"),
    source!("sequencer.sv"),
    source!("controller.sv"),
    source!("controlled_circuit.sv"),
    source!("handshake.sv"),
    source!("link/frame_rx.sv"),
    source!("link/frame_tx.sv"),
];
pub const SIMULATION: [SourceFile; 2] = [source!("sim/top.sv"), source!("sim/steps.vlt")];
pub const BOARD: [SourceFile; 4] = [
    source!("link/uart_rx.sv"),
    source!("link/uart_tx.sv"),
    source!("pll_drp.sv"),
    source!("seven_segment.sv"),
];

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CircuitSource {
    pub package: String,
    pub nodes: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Sources {
    pub directory: PathBuf,
    pub packages: Vec<&'static str>,
    /// What runs the nodes: the `nodes` module and the modules it instantiates, until
    /// `tick::compose` puts the tick in their place.
    pub nodes: Vec<&'static str>,
    pub design: Vec<&'static str>,
}

impl Sources {
    pub fn write<'a>(
        directory: &Path,
        circuit: &'a CircuitSource,
        top: impl IntoIterator<Item = SourceFile<'a>>,
    ) -> io::Result<Sources> {
        let generated = [(CIRCUIT_PACKAGE, &circuit.package), (NODES, &circuit.nodes)];
        let [circuit_package, nodes_module] =
            generated.map(|(path, text)| SourceFile { path, text });
        let packages = vec![PACKAGE, circuit_package];
        let nodes: Vec<_> = [nodes_module]
            .into_iter()
            .chain(NODE_MODULES.iter().copied())
            .collect();
        let design: Vec<_> = DESIGN.iter().copied().chain(top).collect();
        write(
            directory,
            [&packages, &nodes, &design].into_iter().flatten().copied(),
        )?;
        let paths = |sources: Vec<SourceFile>| sources.iter().map(|source| source.path).collect();
        Ok(Sources {
            directory: directory.to_owned(),
            packages: paths(packages),
            nodes: paths(nodes),
            design: paths(design),
        })
    }

    /// In the order in which a tool must read them: a package before the modules that name it.
    pub fn files(&self) -> impl Iterator<Item = &'static str> + '_ {
        let files = [&self.packages, &self.nodes, &self.design];
        files.into_iter().flatten().copied()
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
