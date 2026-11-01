use mchprs_blocks::BlockPos;
use mchprs_redpiler::backend::direct::DirectEngine;
use mchprs_redpiler::{
    BlockMap, CompileProgress, CompilerInput, CompilerOptions, Engine, Input, Netlist, NodeId,
    PendingTick, Reporting,
};
use mchprs_save_data::plot_data::PlotData;
use mchprs_world::testing::TestWorld;
use serde::Deserialize;
use sha2::{Digest, Sha256};
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

pub type Result<T> = std::result::Result<T, Box<dyn Error + Send + Sync>>;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Experiment {
    pub plot: PathBuf,
    #[serde(default)]
    pub origin: [i32; 2],
    #[serde(default)]
    pub options: String,
    pub ticks: u64,
    pub observe: Option<Observation>,
    #[serde(default, rename = "input")]
    pub inputs: Vec<InputAt>,
}

#[derive(Deserialize, Clone, Copy)]
#[serde(deny_unknown_fields)]
pub struct Observation {
    pub from: [i32; 3],
    pub to: [i32; 3],
    pub every: Option<u64>,
}

#[derive(Deserialize, Clone, Copy)]
#[serde(deny_unknown_fields)]
pub struct InputAt {
    pub tick: u64,
    pub position: [i32; 3],
    pub powered: Option<bool>,
}

impl InputAt {
    pub fn position(self) -> BlockPos {
        position(self.position)
    }

    pub fn input(self) -> Input {
        match self.powered {
            Some(powered) => Input::PressurePlate { powered },
            None => Input::Interact,
        }
    }
}

pub fn position([x, y, z]: [i32; 3]) -> BlockPos {
    BlockPos::new(x, y, z)
}

pub struct Loaded {
    pub experiment: Experiment,
    pub path: PathBuf,
    pub plot_sha256: String,
    pub world: TestWorld,
    pub options: CompilerOptions,
}

impl Loaded {
    pub fn open(path: &Path) -> Result<Loaded> {
        let path = std::path::absolute(path)?;
        let mut experiment: Experiment = toml::from_str(&fs::read_to_string(&path)?)?;
        experiment.inputs.sort_by_key(|input| input.tick);
        if experiment
            .inputs
            .iter()
            .any(|input| input.tick > experiment.ticks)
        {
            return Err("an input is scheduled after the last tick".into());
        }
        let plot_path = path.parent().unwrap().join(&experiment.plot);
        let bytes = fs::read(&plot_path)?;
        let plot_sha256 = format!("{:x}", Sha256::digest(&bytes));
        let plot = PlotData::load_from_file(&plot_path)
            .map_err(|error| format!("cannot load {}: {error:?}", plot_path.display()))?;
        let world = world(plot, experiment.origin)?;
        let options = CompilerOptions::parse(&experiment.options);
        Ok(Loaded {
            experiment,
            path,
            plot_sha256,
            world,
            options,
        })
    }

    pub fn compile(&self) -> Compiled {
        let started = Instant::now();
        let input = CompilerInput {
            world: &self.world,
            bounds: self.world.corners(),
            ticks: &self.world.to_be_ticked,
        };
        let netlist = mchprs_redpiler::compile(&input, &self.options, &CompileProgress::default());
        let block_map = BlockMap::new(&netlist);
        let pending_ticks = block_map.map_ticks(&self.world.to_be_ticked);
        Compiled {
            netlist,
            block_map,
            pending_ticks,
            compile_seconds: started.elapsed().as_secs_f64(),
            stages: Default::default(),
        }
    }
}

fn world(mut plot: PlotData, [origin_x, origin_z]: [i32; 2]) -> Result<TestWorld> {
    let width = plot.chunk_data.len().isqrt();
    let height = plot
        .chunk_data
        .first()
        .map_or(0, |chunk| chunk.sections.len());
    if width == 0 || width * width != plot.chunk_data.len() || height == 0 {
        return Err("the plot must be a non-empty square of chunks".into());
    }
    let (width, height) = (width as i32, height as i32);
    for tick in &mut plot.pending_ticks {
        tick.pos = BlockPos::new(tick.pos.x - origin_x, tick.pos.y, tick.pos.z - origin_z);
        let inside = (0..width * 16).contains(&tick.pos.x) && (0..width * 16).contains(&tick.pos.z);
        if !inside {
            return Err(format!(
                "pending tick at {} lies outside the plot; check `origin`",
                tick.pos
            )
            .into());
        }
    }
    let chunks = plot
        .chunk_data
        .into_iter()
        .enumerate()
        .map(|(index, chunk)| chunk.load(index as i32 / width, index as i32 % width))
        .collect();
    let mut world = TestWorld {
        chunks,
        to_be_ticked: plot.pending_ticks,
        x_size: width,
        y_size: height,
        z_size: width,
    };
    let (first_pos, second_pos) = world.corners();
    mchprs_redstone::update_region(&mut world, first_pos, second_pos);
    Ok(world)
}

#[cfg(feature = "fpga")]
pub fn write_verilog(path: &Path, directory: &Path) -> Result<()> {
    let compiled = Loaded::open(path)?.compile();
    mchprs_redpiler_fpga::write_verilog(&compiled.netlist, &compiled.pending_ticks, directory)?;
    eprintln!(
        "{} nodes written to {}",
        compiled.netlist.len(),
        directory.display()
    );
    Ok(())
}

pub struct Compiled {
    pub netlist: Netlist,
    pub block_map: BlockMap,
    pub pending_ticks: Vec<PendingTick>,
    pub compile_seconds: f64,
    pub stages: std::cell::RefCell<Vec<Stage>>,
}

#[derive(serde::Serialize)]
pub struct Stage {
    pub backend: String,
    pub name: &'static str,
    pub seconds: f64,
}

#[derive(Default)]
struct Stages {
    current: Option<(&'static str, Instant)>,
    finished: Vec<Stage>,
}

impl Stages {
    fn observe(&mut self, backend: &str, name: Option<&'static str>) {
        if self.current.map(|(current, _)| current) == name {
            return;
        }
        if let Some((finished, started)) = self.current.take() {
            let seconds = started.elapsed().as_secs_f64();
            eprintln!("{backend}: {finished} took {seconds:.1}s");
            self.finished.push(Stage {
                backend: backend.to_owned(),
                name: finished,
                seconds,
            });
        }
        self.current = name.map(|name| (name, Instant::now()));
    }
}

impl Compiled {
    pub fn engine(&self, backend: &str, reporting: Reporting) -> Result<Box<dyn Engine>> {
        if backend == "direct" {
            return Ok(Box::new(DirectEngine::new(
                &self.netlist,
                reporting,
                &self.pending_ticks,
            )));
        }
        #[cfg(feature = "fpga")]
        if let Ok(target) = backend.parse::<mchprs_redpiler_fpga::Target>() {
            let progress = CompileProgress::default();
            let (netlist, pending_ticks) = (&self.netlist, &self.pending_ticks);
            return std::thread::scope(|scope| {
                let preparing = scope.spawn(|| {
                    mchprs_redpiler_fpga::prepare(
                        target,
                        netlist,
                        pending_ticks,
                        reporting == Reporting::InputsAndOutputs,
                        &progress,
                    )
                });
                let mut stages = Stages::default();
                while !preparing.is_finished() {
                    stages.observe(backend, progress.status().map(|status| status.name));
                    std::thread::sleep(std::time::Duration::from_millis(20));
                }
                stages.observe(backend, None);
                let engine = preparing.join().unwrap()?;
                self.stages.borrow_mut().extend(stages.finished);
                Ok(engine)
            });
        }
        Err(format!("unknown backend {backend}; phase, verilator and fpga need rilc built with --features fpga").into())
    }

    pub fn node_at(&self, input: InputAt) -> Result<NodeId> {
        self.block_map
            .node_accepting(input.position(), input.input())
            .ok_or_else(|| {
                format!(
                    "no node accepting {:?} at {}",
                    input.input(),
                    input.position()
                )
                .into()
            })
    }
}

pub fn run_until(engine: &mut dyn Engine, ticks: u64) -> Result<()> {
    let mut remaining = ticks;
    while remaining != 0 {
        remaining -= engine.run_ticks(
            remaining,
            Instant::now() + std::time::Duration::from_secs(1),
        )?;
    }
    Ok(())
}

pub fn boundaries(experiment: &Experiment, every: Option<u64>) -> Vec<u64> {
    let mut ticks: Vec<u64> = experiment.inputs.iter().map(|input| input.tick).collect();
    if let Some(every) = every.filter(|every| *every != 0) {
        ticks.extend((0..=experiment.ticks).step_by(every as usize));
    }
    ticks.push(0);
    ticks.push(experiment.ticks);
    ticks.sort_unstable();
    ticks.dedup();
    ticks
}

pub fn list_inputs(path: &Path) -> Result<()> {
    let loaded = Loaded::open(path)?;
    let compiled = loaded.compile();
    println!(
        "{} nodes, {} pending ticks",
        compiled.netlist.len(),
        compiled.pending_ticks.len()
    );
    for id in compiled.netlist.ids() {
        let node = compiled.netlist.node(id);
        if node.is_input || node.is_output {
            let role = if node.is_input { "input" } else { "output" };
            for (position, _) in &node.block {
                println!(
                    "{role} {:?} at [{}, {}, {}]",
                    node.ty, position.x, position.y, position.z
                );
            }
        }
    }
    Ok(())
}
