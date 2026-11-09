use crate::input::{Input, InputError};
use crate::sound::Sound;
use crate::tick_queue::{run_scheduled_ticks, PendingTick, TickEntry, TickQueue};
use crate::{behavior, plate, Aabb, Redstone, RedstoneWorld};
use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::BlockPos;
use std::mem;

#[derive(Debug)]
pub struct Interpreter<B: BlockStore> {
    blocks: B,
    redstone: Redstone,
    standing: Vec<BlockPos>,
    sounds: Vec<(BlockPos, Sound)>,
}

impl<B: BlockStore> Interpreter<B> {
    pub fn new(blocks: B, pending: Vec<TickEntry>) -> Interpreter<B> {
        Interpreter {
            blocks,
            redstone: Redstone::new(TickQueue::new(pending)),
            standing: Vec::new(),
            sounds: Vec::new(),
        }
    }

    pub fn take_sounds(&mut self) -> Vec<(BlockPos, Sound)> {
        mem::take(&mut self.sounds)
    }

    pub fn tick(&mut self) {
        run_scheduled_ticks(self);
        for index in 0..self.standing.len() {
            let pos = self.standing[index];
            plate::entity_inside(self, pos);
        }
    }

    pub fn use_block(&mut self, pos: BlockPos) {
        behavior::use_block(self, pos);
    }

    pub fn drive(&mut self, pos: BlockPos, input: Input) -> Result<(), InputError> {
        let unsupported = Err(InputError::Unsupported(input));
        match self.block(pos) {
            block if block.pressure_plate().is_some() => {
                let touched = match input {
                    Input::Toggle => !self.is_standing_on(pos),
                    Input::Touch(touched) => touched,
                    Input::Set(_) | Input::Press => return unsupported,
                };
                if touched {
                    self.stand_on(pos);
                } else {
                    self.step_off(pos);
                }
            }
            block if block.button().is_some() => match input {
                Input::Press => self.use_block(pos),
                _ => return unsupported,
            },
            Block::Lever { powered, .. } => {
                let powers = match input {
                    Input::Toggle => !powered,
                    Input::Set(SignalStrength::Zero) => false,
                    Input::Set(SignalStrength::Fifteen) => true,
                    _ => return unsupported,
                };
                if powers != powered {
                    self.use_block(pos);
                }
            }
            _ => return Err(InputError::NotAnInput),
        }
        Ok(())
    }

    pub fn stand_on(&mut self, pos: BlockPos) {
        if !self.is_standing_on(pos) {
            self.standing.push(pos);
        }
        plate::entity_inside(self, pos);
    }

    pub fn step_off(&mut self, pos: BlockPos) {
        self.standing.retain(|&standing| standing != pos);
    }

    pub fn is_standing_on(&self, pos: BlockPos) -> bool {
        self.standing.contains(&pos)
    }

    pub fn blocks(&self) -> &B {
        &self.blocks
    }

    pub fn blocks_mut(&mut self) -> &mut B {
        &mut self.blocks
    }

    pub fn is_idle(&self) -> bool {
        self.redstone.is_idle()
    }

    pub fn tick_entries(&self) -> &[TickEntry] {
        self.redstone.ticks.entries()
    }

    pub fn pending_ticks(&self) -> Vec<(BlockPos, PendingTick)> {
        let entries = self.tick_entries().iter();
        let entries = entries.filter(|entry| entry.is_for(self.block(entry.pos)));
        let mut pending: Vec<_> = entries
            .map(|entry| {
                let tick = PendingTick {
                    delay: entry.ticks_left,
                    priority: entry.priority,
                };
                (entry.pos, tick)
            })
            .collect();
        pending.sort_by_key(|&(_, tick)| tick);
        pending
    }
}

impl<B: BlockStore> BlockView for Interpreter<B> {
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

impl<B: BlockStore> BlockStore for Interpreter<B> {
    fn set_block(&mut self, pos: BlockPos, block: Block) -> bool {
        self.blocks.set_block(pos, block)
    }

    fn set_block_entity(&mut self, pos: BlockPos, block_entity: Option<BlockEntity>) {
        self.blocks.set_block_entity(pos, block_entity);
    }
}

impl<B: BlockStore> RedstoneWorld for Interpreter<B> {
    fn redstone(&mut self) -> &mut Redstone {
        &mut self.redstone
    }

    fn has_entity_in(&self, area: Aabb) -> bool {
        self.standing
            .iter()
            .any(|&pos| block_box(pos).intersects(&area))
    }

    fn play_sound(&mut self, pos: BlockPos, sound: Sound) {
        self.sounds.push((pos, sound));
    }
}

fn block_box(pos: BlockPos) -> Aabb {
    let min = [pos.x, pos.y, pos.z].map(f64::from);
    Aabb {
        min,
        max: min.map(|coord| coord + 1.0),
    }
}
