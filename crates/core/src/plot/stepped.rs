use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_redstone::input::Input;
use mchprs_redstone::RedstoneWorld;
use mchprs_simulation::backend::{Backend, CompileContext, CompileError, Source};
use mchprs_simulation::block_changes::BlockChanges;
use mchprs_simulation::readback::Readback;
use mchprs_simulation::ticks::SimulationError;
use mchprs_simulation::{Blocks, Simulation, View};
use mchprs_syntax::flag::FlagValues;

#[derive(Debug, Clone, Default)]
pub struct CompilerOptions {
    pub flags: FlagValues,
    pub update: bool,
}

#[derive(Default)]
pub struct Compiler {
    backend: Option<Box<dyn Simulation<Blocks>>>,
    options: CompilerOptions,
}

impl Compiler {
    pub fn is_active(&self) -> bool {
        self.backend.is_some()
    }

    pub fn current_flags(&self) -> Option<&CompilerOptions> {
        self.backend.as_ref().map(|_| &self.options)
    }

    pub fn compile(
        &mut self,
        backend: &dyn Backend,
        source: &Source<'_>,
        options: CompilerOptions,
        context: &mut CompileContext<'_>,
    ) -> Result<(), CompileError> {
        let (compiled, _) = backend.compile(source, &options.flags, context)?;
        self.backend = Some(compiled);
        self.options = options;
        Ok(())
    }

    pub fn reset<W: RedstoneWorld>(
        &mut self,
        world: &mut W,
        bounds: (BlockPos, BlockPos),
    ) -> Result<(), SimulationError> {
        if let Some(compiled) = self.backend.take() {
            let end = compiled.stop()?;
            end.changes.apply(world);
            let Readback::Complete(writeback) = end.state else {
                return Err(anyhow::anyhow!("the backend could not read its state"));
            };
            writeback.apply(world);
        }
        if self.options.update {
            mchprs_redstone::update_region(world, Region::new(bounds.0, bounds.1));
        }
        self.options = CompilerOptions::default();
        Ok(())
    }

    pub fn tick(&mut self) -> Result<(), SimulationError> {
        self.tickn(1)
    }

    pub fn tickn(&mut self, ticks: u64) -> Result<(), SimulationError> {
        self.backend
            .as_mut()
            .expect("compiler is active")
            .run(ticks)?;
        Ok(())
    }

    pub fn on_use_block(
        &mut self,
        pos: BlockPos,
        block: Block,
    ) -> Result<(), mchprs_simulation::input::DriveError> {
        if let Some(input) = Input::of_use(block) {
            self.backend
                .as_mut()
                .expect("compiler is active")
                .drive(pos, input)?;
        }
        Ok(())
    }

    pub fn set_pressure_plate(
        &mut self,
        pos: BlockPos,
        powered: bool,
    ) -> Result<(), mchprs_simulation::input::DriveError> {
        self.backend
            .as_mut()
            .expect("compiler is active")
            .drive(pos, Input::Touch(powered))
    }

    pub fn flush<W: RedstoneWorld>(&mut self, world: &mut W) -> Result<(), SimulationError> {
        let mut changes = BlockChanges::default();
        self.backend
            .as_mut()
            .expect("compiler is active")
            .show(View::Observed, &mut changes)?;
        changes.apply(world);
        Ok(())
    }

    pub fn flush_all<W: RedstoneWorld>(&mut self, world: &mut W) -> Result<(), SimulationError> {
        let mut changes = BlockChanges::default();
        let machine = self.backend.as_mut().expect("compiler is active");
        machine.show(View::All, &mut changes)?;
        changes.apply(world);
        Ok(())
    }
}
