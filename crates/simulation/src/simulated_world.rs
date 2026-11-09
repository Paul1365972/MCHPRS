use crate::block_changes::BlockChanges;
use crate::component::strength_at;
use crate::input::DriveError;
use crate::layout::Layout;
use crate::readback::Readback;
use crate::ticks::{Ran, SimulationError};
use crate::write_back::WriteBack;
use crate::{Blocks, Simulation, View};
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_redstone::input::Input;
use mchprs_redstone::sound::Sound;
use std::mem;

/// A world and the simulation compiled from it, stepped on the caller's thread. After every call
/// the world shows what the simulation observes.
pub struct SimulatedWorld {
    world: BlockVolume,
    layout: Layout,
    simulation: Box<dyn Simulation<Blocks>>,
    sounds: Vec<(BlockPos, Sound)>,
}

impl SimulatedWorld {
    /// `world` holds the blocks the simulation was compiled from.
    pub fn new(
        world: BlockVolume,
        simulation: Box<dyn Simulation<Blocks>>,
        layout: Layout,
    ) -> SimulatedWorld {
        SimulatedWorld {
            world,
            layout,
            simulation,
            sounds: Vec::new(),
        }
    }

    pub fn layout(&self) -> &Layout {
        &self.layout
    }

    pub fn drive(&mut self, pos: BlockPos, input: Input) -> Result<(), DriveError> {
        self.simulation.drive(pos, input)?;
        Ok(self.show()?)
    }

    pub fn run(&mut self, ticks: u64) -> Result<Ran, SimulationError> {
        let ran = self.simulation.run(ticks)?;
        self.show()?;
        Ok(ran)
    }

    pub fn settle(&mut self, limit: u64) -> Result<Ran, SimulationError> {
        let ran = self.simulation.settle(limit)?;
        self.show()?;
        Ok(ran)
    }

    pub fn is_current(&self, pos: BlockPos) -> bool {
        match self.layout.at(pos) {
            Some(component) => component.observed,
            None => strength_at(&self.world, pos).is_none(),
        }
    }

    pub fn take_sounds(&mut self) -> Vec<(BlockPos, Sound)> {
        mem::take(&mut self.sounds)
    }

    pub fn read(&mut self) -> Result<Readback<WriteBack>, SimulationError> {
        self.simulation.read()
    }

    pub fn stop(self) -> Result<(BlockVolume, Readback<WriteBack>), SimulationError> {
        let mut world = self.world;
        let end = self.simulation.stop()?;
        end.changes.write(&mut world);
        Ok((world, end.state))
    }

    fn show(&mut self) -> Result<(), SimulationError> {
        let mut changes = BlockChanges::default();
        self.simulation.show(View::Observed, &mut changes)?;
        changes.write(&mut self.world);
        self.sounds.append(&mut changes.sounds);
        Ok(())
    }
}

impl BlockView for SimulatedWorld {
    fn block(&self, pos: BlockPos) -> Block {
        self.world.block(pos)
    }

    fn block_entity(&self, pos: BlockPos) -> Option<&BlockEntity> {
        self.world.block_entity(pos)
    }

    fn occupied_regions(&self, region: Region) -> Vec<Region> {
        self.world.occupied_regions(region)
    }
}
