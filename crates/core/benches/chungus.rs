use std::path::Path;

use criterion::*;
use mchprs_blocks::BlockPos;
use mchprs_core::plot::{PlotWorld, PLOT_WIDTH};
use mchprs_core::redpiler::{Compiler, CompilerOptions};
use mchprs_core::world::storage::Chunk;
use mchprs_save_data::plot_data::PlotData;

const START_BUTTON: BlockPos = BlockPos::new(187, 99, 115);

fn load_world(path: impl AsRef<Path>) -> PlotWorld {
    let data = PlotData::load_from_file(path).unwrap();

    let chunks: Vec<Chunk> = data
        .chunk_data
        .into_iter()
        .enumerate()
        .map(|(i, c)| Chunk::load(i as i32 / PLOT_WIDTH, i as i32 % PLOT_WIDTH, c))
        .collect();
    PlotWorld {
        x: 0,
        z: 0,
        chunks,
        to_be_ticked: data.pending_ticks,
        packet_senders: Vec::new(),
    }
}

fn init_compiler() -> Compiler {
    let mut world = load_world("./benches/chungus_mandelbrot_plot");
    let mut compiler: Compiler = Default::default();

    let options = CompilerOptions::parse("-O");
    let bounds = world.get_corners();
    compiler.compile(&mut world, bounds, options, Vec::new());
    compiler.on_use_block(START_BUTTON);
    compiler
}

fn chungus_mandelbrot(c: &mut Criterion) {
    let mut compiler = init_compiler();

    c.bench_function("chungus-mandelbrot-tick", |b| {
        b.iter(|| compiler.tick());
    });
}

criterion_group!(chungus, chungus_mandelbrot);
criterion_main!(chungus);
