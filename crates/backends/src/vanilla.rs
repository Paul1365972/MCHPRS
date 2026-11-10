use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_redstone::input::{is_drivable, Input, InputError};
use mchprs_redstone::interpreter::Interpreter;
use mchprs_redstone::{noteblock, plate};
use mchprs_simulation::backend::{
    Backend, CompileContext, CompileError, CompileStep, PortError, Source,
};
use mchprs_simulation::block_changes::BlockChanges;
use mchprs_simulation::component::{default_role, powered_at, strength_at};
use mchprs_simulation::input::DriveError;
use mchprs_simulation::layout::{Component, Layout};
use mchprs_simulation::ports::Role;
use mchprs_simulation::readback::Readback;
use mchprs_simulation::stepper;
use mchprs_simulation::ticks::{run_until_hit, settle_until_hit, Ran, SimulationError};
use mchprs_simulation::watch::{Breakpoints, Hit, Watch};
use mchprs_simulation::write_back::WriteBack;
use mchprs_simulation::{Blocks, Capabilities, End, Running, Simulation, View};
use mchprs_syntax::flag::{Flag, FlagValues};
use rustc_hash::{FxHashMap, FxHashSet};

#[derive(Debug, Clone, Copy, Default)]
pub struct VanillaBackend;

impl Backend for VanillaBackend {
    fn name(&self) -> &str {
        "vanilla"
    }

    fn flags(&self) -> &[Flag] {
        &[]
    }

    fn compile(
        &self,
        source: &Source<'_>,
        _: &FlagValues,
        context: &mut CompileContext<'_>,
    ) -> Result<(Box<dyn Simulation<Blocks>>, Layout), CompileError> {
        context.report(CompileStep::single("Copying the world"))?;
        let blocks = BlockVolume::copy(&source.blocks, source.region);
        let components = find_components(&blocks, source)?;
        let inputs = components
            .iter()
            .filter(|(_, component)| component.port == Some(Role::Input))
            .map(|&(pos, _)| pos)
            .collect();
        let log = ChangeLog {
            blocks,
            changed: Vec::new(),
            changed_entities: Vec::new(),
        };
        let mut interpreter = Interpreter::new(log, source.ticks.to_vec());
        plate::recheck_pressed_plates(&mut interpreter, source.region);
        let simulation = VanillaSimulation {
            interpreter,
            region: source.region,
            inputs,
            ticks_run: 0,
            breakpoints: Breakpoints::default(),
        };
        Ok((Box::new(simulation), components.into_iter().collect()))
    }
}

fn find_components(
    blocks: &BlockVolume,
    source: &Source<'_>,
) -> Result<Vec<(BlockPos, Component)>, PortError> {
    for &(pos, role) in source.declared {
        if strength_at(blocks, pos).is_none() {
            return Err(PortError::NoComponent(pos));
        }
        if role == Role::Input && !is_drivable(blocks.block(pos)) {
            return Err(PortError::NotDrivable(pos));
        }
    }
    let declared: FxHashMap<BlockPos, Role> = source.declared.iter().copied().collect();
    let components = blocks
        .occupied_positions(source.region)
        .filter(|&pos| strength_at(blocks, pos).is_some())
        .map(|pos| {
            let role = declared.get(&pos).copied();
            let port = role.or_else(|| default_role(blocks.block(pos)));
            (
                pos,
                Component {
                    port,
                    observed: true,
                },
            )
        });
    Ok(components.collect())
}

struct ChangeLog {
    blocks: BlockVolume,
    changed: Vec<BlockPos>,
    changed_entities: Vec<BlockPos>,
}

impl ChangeLog {
    fn show(&mut self, changes: &mut BlockChanges) {
        self.unshown(changes);
        self.changed.clear();
        self.changed_entities.clear();
    }

    fn unshown(&mut self, changes: &mut BlockChanges) {
        self.changed.sort_unstable();
        self.changed.dedup();
        for &pos in &self.changed {
            changes.blocks.push((pos, self.blocks.block(pos)));
        }
        self.changed_entities.sort_unstable();
        self.changed_entities.dedup();
        for &pos in &self.changed_entities {
            let block_entity = self.blocks.block_entity(pos).cloned();
            changes.block_entities.push((pos, block_entity));
        }
    }
}

impl BlockView for ChangeLog {
    fn block(&self, pos: BlockPos) -> Block {
        self.blocks.block(pos)
    }

    fn block_entity(&self, pos: BlockPos) -> Option<&BlockEntity> {
        self.blocks.block_entity(pos)
    }

    fn occupied_regions(&self, region: Region) -> Vec<Region> {
        self.blocks.occupied_regions(region)
    }
}

impl BlockStore for ChangeLog {
    fn set_block(&mut self, pos: BlockPos, block: Block) -> bool {
        let changed = self.blocks.set_block(pos, block);
        if changed {
            self.changed.push(pos);
        }
        changed
    }

    fn set_block_entity(&mut self, pos: BlockPos, block_entity: Option<BlockEntity>) {
        self.blocks.set_block_entity(pos, block_entity);
        self.changed_entities.push(pos);
    }
}

struct VanillaSimulation {
    interpreter: Interpreter<ChangeLog>,
    region: Region,
    inputs: FxHashSet<BlockPos>,
    ticks_run: u64,
    breakpoints: Breakpoints<BlockPos>,
}

impl VanillaSimulation {
    fn tick(&mut self) -> Option<Hit> {
        self.interpreter.tick();
        self.ticks_run += 1;
        let world = &self.interpreter;
        let powered = |pos| powered_at(world, pos) == Some(true);
        self.breakpoints.fire_all(self.ticks_run, powered)
    }

    fn is_idle(&self) -> bool {
        self.interpreter.is_idle()
    }
}

impl Simulation<Blocks> for VanillaSimulation {
    fn capabilities(&self) -> Capabilities {
        Capabilities {
            breakpoints: true,
            settling: true,
            write_back: true,
        }
    }

    fn ticks(&self) -> u64 {
        self.ticks_run
    }

    fn run(&mut self, ticks: u64) -> Result<Ran, SimulationError> {
        Ok(run_until_hit(self, ticks, VanillaSimulation::tick))
    }

    fn settle(&mut self, limit: u64) -> Result<Ran, SimulationError> {
        let ran = settle_until_hit(
            self,
            limit,
            VanillaSimulation::is_idle,
            VanillaSimulation::tick,
        );
        Ok(ran)
    }

    fn drive(&mut self, pos: BlockPos, input: Input) -> Result<(), DriveError> {
        if !self.inputs.contains(&pos) {
            return Err(InputError::NotAnInput.into());
        }
        Ok(self.interpreter.drive(pos, input)?)
    }

    fn watch(&mut self, watch: Watch<BlockPos>) -> Result<(), SimulationError> {
        let world = &self.interpreter;
        let powered = |pos| powered_at(world, pos) == Some(true);
        self.breakpoints = Breakpoints::new(watch.breakpoints, Some(&self.breakpoints), powered);
        Ok(())
    }

    fn show(&mut self, _: View, changes: &mut BlockChanges) -> Result<(), SimulationError> {
        self.interpreter.blocks_mut().show(changes);
        changes.sounds.extend(self.interpreter.take_sounds());
        Ok(())
    }

    fn read(&mut self) -> Result<Readback<WriteBack>, SimulationError> {
        let mut changes = BlockChanges::default();
        self.interpreter.blocks_mut().unshown(&mut changes);
        Ok(Readback::Complete(WriteBack {
            changes,
            ticks: self.interpreter.pending_ticks(),
            region: self.region,
        }))
    }

    fn stop(mut self: Box<Self>) -> Result<End<Blocks>, SimulationError> {
        noteblock::run_block_events(&mut self.interpreter);
        let mut changes = BlockChanges::default();
        self.show(View::All, &mut changes)?;
        Ok(End {
            changes,
            state: Readback::Complete(WriteBack {
                changes: BlockChanges::default(),
                ticks: self.interpreter.pending_ticks(),
                region: self.region,
            }),
        })
    }

    fn start(self: Box<Self>) -> Result<Box<dyn Running<Blocks>>, SimulationError> {
        stepper::start(self, "vanilla")
    }
}
