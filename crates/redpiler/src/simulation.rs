use crate::block_map::{BlockChanges, BlockMap};
use crate::circuit::Circuit;
use crate::engine::{Changes, DriveError, Engine, EngineError, Input, Snapshot};
use mchprs_blocks::BlockPos;
use mchprs_redstone::tick_queue::TickEntry;
use std::sync::Arc;

pub struct Simulation<E: Engine = Box<dyn Engine>> {
    circuit: Arc<Circuit>,
    engine: E,
    blocks: BlockMap,
}

impl<E: Engine> Simulation<E> {
    pub fn new(circuit: Arc<Circuit>, engine: E) -> Simulation<E> {
        Simulation {
            blocks: BlockMap::new(&circuit),
            circuit,
            engine,
        }
    }

    pub fn circuit(&self) -> &Arc<Circuit> {
        &self.circuit
    }

    pub fn drive(&mut self, pos: BlockPos, input: Input) -> Result<(), DriveError> {
        let node = self.circuit.node_at(pos).ok_or(DriveError::NoNode(pos))?;
        self.circuit.check_input(node, input)?;
        self.engine.drive(node, input).map_err(DriveError::Engine)
    }

    pub fn run(&mut self, ticks: u64) -> Result<(), EngineError> {
        self.engine.run(ticks)
    }

    pub fn sync(&mut self) -> Result<BlockChanges, EngineError> {
        self.record_changes()?;
        Ok(self.blocks.take_changes(&self.circuit))
    }

    pub fn sync_all(&mut self) -> Result<BlockChanges, EngineError> {
        self.record_changes()?;
        self.blocks.record_snapshot(&self.engine.snapshot()?);
        Ok(self.blocks.take_changes(&self.circuit))
    }

    pub fn pending_ticks(&mut self) -> Result<Vec<TickEntry>, EngineError> {
        let snapshot = self.engine.snapshot()?;
        Ok(self.blocks.tick_entries(&snapshot.pending_ticks))
    }

    pub fn snapshot(&mut self) -> Result<Snapshot, EngineError> {
        self.engine.snapshot()
    }

    pub fn restore(&mut self, snapshot: &Snapshot) -> Result<(), EngineError> {
        self.circuit.check_snapshot(snapshot)?;
        self.record_changes()?;
        self.engine.restore(snapshot)
    }

    pub fn reset(&mut self) -> Result<(), EngineError> {
        self.record_changes()?;
        self.engine.reset()
    }

    fn record_changes(&mut self) -> Result<(), EngineError> {
        let mut changes = Changes::default();
        self.engine.changes(&mut changes)?;
        self.blocks.record(changes);
        Ok(())
    }
}
