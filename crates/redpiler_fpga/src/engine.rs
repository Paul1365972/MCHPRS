use crate::layout::StateLayout;
use crate::link::Controller;
use crate::protocol::{Contents, Mode, Readout};
use crate::running;
use crate::verilog::Design;
use anyhow::{anyhow, ensure};
use mchprs_redpiler::circuit::{Circuit, InputKinds, NodeId};
use mchprs_redpiler::graph::NodeState;
use mchprs_redpiler::nodes::{Changes, Nodes, Snapshot};
use mchprs_redstone::input::Input;
use mchprs_redstone::tick_queue::TickPriority;
use mchprs_simulation::input::DriveError;
use mchprs_simulation::readback::Readback;
use mchprs_simulation::ticks::{Ran, SimulationError, Stop};
use mchprs_simulation::watch::Watch;
use mchprs_simulation::{Capabilities, End, Running, Simulation, View};
use std::mem;
use std::thread;
use std::time::Duration;

#[derive(Debug)]
pub struct FpgaEngine {
    controller: Controller,
    layout: StateLayout,
    input_kinds: InputKinds,
    observed: Vec<bool>,
    reported_states: Vec<NodeState>,
    unreported: Changes,
    clock_hertz: f64,
    ticks: u64,
}

impl FpgaEngine {
    pub fn connect(
        mut controller: Controller,
        circuit: &Circuit,
        design: &Design,
        clock_hertz: f64,
    ) -> Result<FpgaEngine, SimulationError> {
        let hello = controller.hello()?;
        ensure!(
            hello.build_nonce == design.build_nonce,
            "the board holds the build {:08x}, not {:08x}",
            hello.build_nonce,
            design.build_nonce
        );
        let initial = circuit.initial();
        let mut engine = FpgaEngine {
            controller,
            layout: design.layout.clone(),
            input_kinds: circuit.input_kinds(),
            observed: circuit
                .ids()
                .map(|node| circuit.node(node).observed)
                .collect(),
            reported_states: initial.states.iter().map(|state| state.shown()).collect(),
            unreported: Changes::default(),
            clock_hertz,
            ticks: 0,
        };
        let readout = engine.controller.read(Contents::Shown)?;
        let shown = engine.layout.decode_shown(&readout.state)?;
        let shows_initial_states = shown
            .states
            .iter()
            .all(|&(node, state)| state == engine.reported_states[node.index()]);
        let mut snapshot = engine.read_snapshot()?;
        let mut expected_ticks = initial.pending_ticks;
        snapshot.pending_ticks.sort();
        expected_ticks.sort();
        let holds_initial_state =
            (snapshot.states, snapshot.pending_ticks) == (initial.states, expected_ticks);
        ensure!(
            shows_initial_states && holds_initial_state,
            "the board does not start in the state of the circuit"
        );
        Ok(engine)
    }

    pub fn clock_hertz(&self) -> f64 {
        self.clock_hertz
    }

    pub fn run_time(&self, ticks: u64) -> Duration {
        let clocks = ticks as f64 * TickPriority::ALL.len() as f64;
        Duration::try_from_secs_f64(clocks / self.clock_hertz).unwrap_or(Duration::MAX)
    }

    pub fn set_mode(&mut self, mode: Mode) -> Result<(), SimulationError> {
        self.ticks = self.controller.set_mode(mode)?.ticks;
        Ok(())
    }

    fn record_shown(&mut self, readout: &Readout) -> Result<(), SimulationError> {
        self.ticks = readout.ticks;
        let bytes = self.layout.bits(Contents::Shown).div_ceil(8);
        let state = readout
            .state
            .get(..bytes)
            .ok_or_else(|| anyhow!("the shown state is missing"))?;
        let shown = self.layout.decode_shown(state)?;
        for (node, state) in shown.states {
            let reported = &mut self.reported_states[node.index()];
            if self.observed[node.index()] && state != *reported {
                self.unreported.states.push((node, state));
                *reported = state;
            }
        }
        self.unreported.played_note_blocks.extend(
            shown
                .played_note_blocks
                .into_iter()
                .filter(|node| self.observed[node.index()]),
        );
        Ok(())
    }

    pub fn take_changes(&mut self) -> Changes {
        let mut changes = mem::take(&mut self.unreported);
        changes.played_note_blocks.sort();
        changes.played_note_blocks.dedup();
        changes
    }

    pub fn read_snapshot(&mut self) -> Result<Snapshot, SimulationError> {
        let readout = self.controller.read(Contents::All)?;
        self.record_shown(&readout)?;
        self.layout.decode_snapshot(&readout.state)
    }
}

impl Simulation<Nodes> for FpgaEngine {
    fn capabilities(&self) -> Capabilities {
        Capabilities {
            breakpoints: false,
            settling: true,
            write_back: true,
        }
    }

    fn ticks(&self) -> u64 {
        self.ticks
    }

    fn run(&mut self, ticks: u64) -> Result<Ran, SimulationError> {
        self.set_mode(Mode::counted(ticks)?)?;
        let start = self.ticks;
        thread::sleep(self.run_time(ticks));
        while self.ticks.wrapping_sub(start) < ticks {
            let readout = self.controller.read(Contents::Shown)?;
            self.record_shown(&readout)?;
        }
        Ok(Ran { ticks, stop: None })
    }

    /// Runs up to twice the ticks it takes until no tick is pending.
    fn settle(&mut self, limit: u64) -> Result<Ran, SimulationError> {
        let mut ticks = 0;
        let mut chunk = 1;
        loop {
            let snapshot = self.read_snapshot()?;
            if snapshot.pending_ticks.is_empty() {
                return Ok(Ran {
                    ticks,
                    stop: Some(Stop::Settled),
                });
            }
            if ticks == limit {
                return Ok(Ran { ticks, stop: None });
            }
            chunk = chunk.min(limit - ticks);
            self.run(chunk)?;
            ticks += chunk;
            chunk = chunk.saturating_mul(2);
        }
    }

    fn drive(&mut self, node: NodeId, input: Input) -> Result<(), DriveError> {
        self.input_kinds.accept(node, input)?;
        let readout = self.controller.drive(node.index() as u32, input)?;
        Ok(self.record_shown(&readout)?)
    }

    fn watch(&mut self, _: Watch<NodeId>) -> Result<(), SimulationError> {
        Ok(())
    }

    fn show(&mut self, view: View, changes: &mut Changes) -> Result<(), SimulationError> {
        if view == View::All {
            let snapshot = self.read_snapshot()?;
            for (index, state) in snapshot.states.into_iter().enumerate() {
                let reported = &mut self.reported_states[index];
                if state.shown() != *reported {
                    *reported = state.shown();
                    self.unreported.states.push((NodeId::new(index), state));
                }
            }
        } else {
            let readout = self.controller.read(Contents::Shown)?;
            self.record_shown(&readout)?;
        }
        changes.extend([self.take_changes()]);
        Ok(())
    }

    fn read(&mut self) -> Result<Readback<Snapshot>, SimulationError> {
        self.read_snapshot().map(Readback::Complete)
    }

    fn stop(mut self: Box<Self>) -> Result<End<Nodes>, SimulationError> {
        let state = Readback::Complete(self.read_snapshot()?);
        let mut changes = Changes::default();
        self.show(View::Observed, &mut changes)?;
        Ok(End { changes, state })
    }

    fn start(self: Box<Self>) -> Result<Box<dyn Running<Nodes>>, SimulationError> {
        running::start(*self)
    }
}
