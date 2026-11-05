use crate::block_map::{BlockChanges, BlockMap};
use crate::circuit::{Circuit, NodeId};
use crate::engine::{Changes, DriveError, Engine, EngineError, Input, Snapshot};
use mchprs_blocks::BlockPos;

pub struct Simulation<E: Engine = Box<dyn Engine>> {
    blocks: BlockMap,
    engine: E,
}

impl<E: Engine> Simulation<E> {
    pub fn new(circuit: Circuit, engine: E) -> Simulation<E> {
        Simulation {
            blocks: BlockMap::new(circuit),
            engine,
        }
    }

    pub fn circuit(&self) -> &Circuit {
        self.blocks.circuit()
    }

    pub fn drive(&mut self, position: BlockPos, input: Input) -> Result<(), DriveError> {
        let node = self
            .circuit()
            .node_at(position)
            .ok_or(DriveError::NoNode(position))?;
        self.drive_node(node, input)
    }

    pub fn drive_node(&mut self, node: NodeId, input: Input) -> Result<(), DriveError> {
        let drive = self.circuit().check_input(node, input)?;
        self.engine.drive(drive).map_err(DriveError::Engine)
    }

    pub fn run(&mut self, ticks: u64) -> Result<(), EngineError> {
        self.engine.run(ticks)
    }

    pub fn settle(&mut self, limit: u64) -> Result<Option<u64>, EngineError> {
        self.engine.settle(limit)
    }

    pub fn sync(&mut self) -> Result<BlockChanges, EngineError> {
        self.record_changes()?;
        Ok(self.blocks.take_changes())
    }

    pub fn sync_all(&mut self) -> Result<BlockChanges, EngineError> {
        self.record_changes()?;
        self.blocks.record_snapshot(&self.engine.snapshot()?);
        Ok(self.blocks.take_changes())
    }

    pub fn snapshot(&mut self) -> Result<Snapshot, EngineError> {
        self.engine.snapshot()
    }

    pub fn restore(&mut self, snapshot: &Snapshot) -> Result<(), EngineError> {
        self.circuit().check_snapshot(snapshot)?;
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
        self.blocks.record(&changes);
        Ok(())
    }
}
