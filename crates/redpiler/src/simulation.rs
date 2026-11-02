use crate::block_map::{BlockChanges, BlockMap};
use crate::circuit::{Circuit, NodeId};
use crate::engine::{DriveError, Engine, EngineError, Input, Snapshot};
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_blocks::BlockPos;
use mchprs_redstone::tick_queue::TickEntry;
use std::sync::Arc;

pub struct Simulation<E: Engine = Box<dyn Engine>> {
    circuit: Arc<Circuit>,
    engine: E,
    blocks: BlockMap,
}

impl<E: Engine> Simulation<E> {
    pub fn new(engine: E) -> Simulation<E> {
        let circuit = engine.circuit().clone();
        Simulation {
            blocks: BlockMap::new(&circuit),
            circuit,
            engine,
        }
    }

    pub fn circuit(&self) -> &Arc<Circuit> {
        &self.circuit
    }

    pub fn drive(&mut self, position: BlockPos, input: Input) -> Result<(), DriveError> {
        let node = self
            .circuit
            .node_at(position)
            .ok_or(DriveError::NoNode(position))?;
        self.drive_node(node, input)
    }

    pub fn drive_node(&mut self, node: NodeId, input: Input) -> Result<(), DriveError> {
        self.circuit.check_input(node, input)?;
        self.engine.drive(node, input).map_err(DriveError::Engine)
    }

    pub fn run(&mut self, ticks: u64) -> Result<(), EngineError> {
        self.engine.run(ticks)
    }

    pub fn is_idle(&mut self) -> Result<bool, EngineError> {
        self.engine.is_idle()
    }

    pub fn settle(&mut self, limit: u64) -> Result<Option<u64>, EngineError> {
        self.engine.settle(limit)
    }

    pub fn sync(&mut self) -> Result<BlockChanges, EngineError> {
        self.blocks.record(self.engine.changes()?);
        Ok(self.blocks.take_changes(&self.circuit))
    }

    pub fn sync_all(&mut self) -> Result<BlockChanges, EngineError> {
        self.blocks.record(self.engine.changes()?);
        self.blocks.record_snapshot(&self.engine.snapshot()?);
        Ok(self.blocks.take_changes(&self.circuit))
    }

    pub fn signal_at(&mut self, position: BlockPos) -> Result<Option<SignalStrength>, EngineError> {
        let Some(node) = self.circuit.node_at(position) else {
            return Ok(None);
        };
        if !self.circuit.node(node).observed {
            return Ok(None);
        }
        self.blocks.record(self.engine.changes()?);
        Ok(Some(self.blocks.state(node).power))
    }

    pub fn pending_ticks(&mut self) -> Result<Vec<TickEntry>, EngineError> {
        let snapshot = self.engine.snapshot()?;
        Ok(self.blocks.tick_entries(&snapshot.pending_ticks))
    }

    pub fn snapshot(&mut self) -> Result<Snapshot, EngineError> {
        self.engine.snapshot()
    }

    pub fn restore(&mut self, snapshot: &Snapshot) -> Result<(), EngineError> {
        self.blocks.record(self.engine.changes()?);
        self.engine.restore(snapshot)
    }

    pub fn reset(&mut self) -> Result<(), EngineError> {
        self.restore(&self.circuit.initial())
    }
}
