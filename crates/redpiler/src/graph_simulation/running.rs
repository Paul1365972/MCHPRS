use super::block_map::BlockMap;
use crate::nodes::Nodes;
use mchprs_blocks::BlockPos;
use mchprs_redstone::input::InputError;
use mchprs_simulation::block_changes::BlockChanges;
use mchprs_simulation::ticks::SimulationError;
use mchprs_simulation::{Blocks, Capabilities, Command, End, Event, Running, Update};
use std::mem;

/// A circuit's running simulation, addressed by the positions of its blocks. A drive at a
/// position without a component is refused at once.
pub(super) struct GraphRunning {
    running: Box<dyn Running<Nodes>>,
    blocks: BlockMap,
    refused: Vec<BlockPos>,
    ticks: u64,
    behind: bool,
}

impl GraphRunning {
    pub(super) fn new(
        running: Box<dyn Running<Nodes>>,
        blocks: BlockMap,
        ticks: u64,
    ) -> GraphRunning {
        GraphRunning {
            running,
            blocks,
            refused: Vec::new(),
            ticks,
            behind: false,
        }
    }

    fn on_blocks(&mut self, update: Option<Update<Nodes>>) -> Update<Blocks> {
        let refused = mem::take(&mut self.refused).into_iter();
        let mut events: Vec<_> = refused
            .map(|pos| Event::Refused(pos, InputError::NotAnInput))
            .collect();
        let mut changes = BlockChanges::default();
        if let Some(update) = update {
            self.blocks.record(&update.changes, &mut changes);
            self.ticks = update.ticks;
            self.behind = update.behind;
            events.extend(update.events.into_iter().filter_map(|event| {
                Some(match event {
                    Event::RunFinished => Event::RunFinished,
                    Event::RunInterrupted => Event::RunInterrupted,
                    Event::Halted(hit) => Event::Halted(hit),
                    Event::Refused(node, error) => {
                        Event::Refused(self.blocks.position(node)?, error)
                    }
                    Event::Shown(view) => Event::Shown(view),
                })
            }));
        }
        Update {
            changes,
            ticks: self.ticks,
            events,
            behind: self.behind,
        }
    }
}

impl Running<Blocks> for GraphRunning {
    fn capabilities(&self) -> Capabilities {
        self.running.capabilities()
    }

    fn send(&mut self, command: Command<BlockPos>) {
        let command = match command {
            Command::Pause => Command::Pause,
            Command::Run(run) => Command::Run(run),
            Command::Drive(pos, input) => match self.blocks.node_at(pos) {
                Some(node) => Command::Drive(node, input),
                None => return self.refused.push(pos),
            },
            Command::Watch(watch) => Command::Watch(self.blocks.watch(watch)),
            Command::Show(view) => Command::Show(view),
        };
        self.running.send(command);
    }

    fn poll(&mut self) -> Result<Option<Update<Blocks>>, SimulationError> {
        let update = self.running.poll()?;
        if update.is_none() && self.refused.is_empty() {
            return Ok(None);
        }
        Ok(Some(self.on_blocks(update)))
    }

    fn wait(&mut self) -> Result<Update<Blocks>, SimulationError> {
        let update = match self.refused.is_empty() {
            true => Some(self.running.wait()?),
            false => None,
        };
        Ok(self.on_blocks(update))
    }

    fn stop(self: Box<Self>) -> Result<End<Blocks>, SimulationError> {
        let GraphRunning {
            running,
            mut blocks,
            ..
        } = *self;
        Ok(blocks.end(running.stop()?))
    }
}
