use anyhow::{Context, Result};
use mchprs_backends::vanilla::VanillaBackend;
use mchprs_backends::Setup;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::BlockPos;
use mchprs_redstone::input::Input;
use mchprs_redstone::interpreter::Interpreter;
use mchprs_redstone::settle;
use mchprs_simulation::backend::Source;
use mchprs_simulation::readback::Readback;
use std::fs;

const LEVER: BlockPos = BlockPos::new(0, 1, 0);
const LAMP: BlockPos = BlockPos::new(2, 1, 0);

fn main() -> Result<()> {
    let bytes = fs::read(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/lamp.schem"))?;
    let volume = mchprs_schematic::read(&bytes)?.volume;
    let region = volume.region();
    let mut world = Interpreter::new(volume, Vec::new());
    settle(&mut world, region, 1000).context("the build never settles")?;

    let vanilla = Setup::new(&VanillaBackend, Default::default());
    for setup in [vanilla, Setup::parse("direct -o")?] {
        let mut simulated_world = setup.simulate(&Source::new(&world, region))?;
        simulated_world.drive(LEVER, Input::Toggle)?;
        for tick in 1..=2 {
            simulated_world.run(1)?;
            let lit = simulated_world.block(LAMP) == Block::RedstoneLamp { lit: true };
            println!("{setup}, tick {tick}: lamp lit: {lit}");
        }
        let (blocks, Readback::Complete(write_back)) = simulated_world.stop()? else {
            anyhow::bail!("{setup} could not read every component back");
        };
        let mut resumed = Interpreter::new(blocks, Vec::new());
        write_back.apply(&mut resumed);
        resumed.tick();
    }
    Ok(())
}
