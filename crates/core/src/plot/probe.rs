use super::scoreboard::ProbeStatus;
use super::simulation::PlotSimulation;
use super::Plot;
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::{BlockFacing, BlockPos};
use mchprs_simulation::component::powered_at;
use mchprs_simulation::layout::Layout;
use mchprs_simulation::watch::{Bit, Breakpoint, BreakpointId, Breakpoints, Condition, Hit, Watch};
use rustc_hash::FxHashMap;
use std::mem;
use std::slice;

pub const MAX_PROBE_BITS: usize = u64::BITS as usize;

pub struct Probe {
    pub name: String,
    positions: Box<[BlockPos]>,
    pub breakpoint: Option<ProbeBreakpoint>,
}

impl Probe {
    pub fn new(
        name: String,
        mut positions: Vec<BlockPos>,
        direction: BlockFacing,
    ) -> Result<Probe, InvalidWidth> {
        check_width(positions.len())?;
        let step = direction.offset_pos(BlockPos::zero(), 1);
        positions.sort_by_key(|pos| {
            let along = pos.x * step.x + pos.y * step.y + pos.z * step.z;
            (along, pos.y, pos.x, pos.z)
        });
        Ok(Probe {
            name,
            positions: positions.into_boxed_slice(),
            breakpoint: None,
        })
    }

    pub fn positions(&self) -> &[BlockPos] {
        &self.positions
    }

    pub fn width(&self) -> usize {
        self.positions.len()
    }

    pub fn read(&self, world: &impl BlockView) -> Option<u64> {
        self.positions
            .iter()
            .enumerate()
            .try_fold(0, |value, (bit, &pos)| {
                Some(value | u64::from(powered_at(world, pos)?) << bit)
            })
    }

    pub fn format(&self, value: u64) -> String {
        format_value(value, self.positions.len())
    }

    pub fn describe(&self, condition: Condition) -> String {
        match condition {
            Condition::Becomes(value) => format!("{} becomes {}", self.name, self.format(value)),
            Condition::Leaves(value) => format!("{} leaves {}", self.name, self.format(value)),
            Condition::Changes => format!("{} changes", self.name),
        }
    }

    fn bits(&self, layout: &Layout, world: &impl BlockView) -> Box<[Bit<BlockPos>]> {
        self.positions
            .iter()
            .map(|&pos| match layout.at(pos) {
                Some(_) => Bit::At(pos),
                None => Bit::Fixed(powered_at(world, pos) == Some(true)),
            })
            .collect()
    }
}

fn format_value(value: u64, width: usize) -> String {
    format!("0x{value:0digits$X}", digits = width.div_ceil(4))
}

fn check_width(width: usize) -> Result<(), InvalidWidth> {
    match width {
        0 => Err(InvalidWidth::Empty),
        1..=MAX_PROBE_BITS => Ok(()),
        width => Err(InvalidWidth::TooWide(width)),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InvalidWidth {
    Empty,
    TooWide(usize),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProbeBreakpoint {
    pub id: BreakpointId,
    pub condition: Condition,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProbeError {
    NameTaken,
    NoProbe,
}

struct HaltLabel {
    name: String,
    condition: Condition,
    width: usize,
}

impl HaltLabel {
    fn describe_hit(&self, value: u64) -> String {
        let name = &self.name;
        let value = format_value(value, self.width);
        match self.condition {
            Condition::Becomes(_) => format!("{name} became {value}"),
            Condition::Leaves(left) => {
                let left = format_value(left, self.width);
                format!("{name} changed from {left} to {value}")
            }
            Condition::Changes => format!("{name} changed to {value}"),
        }
    }
}

#[derive(Default)]
pub(super) struct Probes {
    probes: Vec<Probe>,
    next_breakpoint_id: u32,
    halt_labels: FxHashMap<BreakpointId, HaltLabel>,
    world_breakpoints: Breakpoints<BlockPos>,
}

impl Probes {
    pub(super) fn iter(&self) -> slice::Iter<'_, Probe> {
        self.probes.iter()
    }

    pub(super) fn world_breakpoints(&mut self) -> &mut Breakpoints<BlockPos> {
        &mut self.world_breakpoints
    }

    fn get(&self, name: &str) -> Option<&Probe> {
        self.probes.iter().find(|probe| probe.name == name)
    }

    fn add(&mut self, probe: Probe) -> Result<(), ProbeError> {
        if self.get(&probe.name).is_some() {
            return Err(ProbeError::NameTaken);
        }
        self.probes.push(probe);
        Ok(())
    }

    fn remove(&mut self, name: &str) -> Result<(), ProbeError> {
        let index = self
            .probes
            .iter()
            .position(|probe| probe.name == name)
            .ok_or(ProbeError::NoProbe)?;
        self.probes.remove(index);
        Ok(())
    }

    fn clear(&mut self) -> usize {
        let count = self.probes.len();
        self.probes.clear();
        count
    }

    fn set_breakpoint(&mut self, name: &str, condition: Option<Condition>) {
        let Some(probe) = self.probes.iter_mut().find(|probe| probe.name == name) else {
            return;
        };
        probe.breakpoint = condition.map(|condition| {
            let id = BreakpointId(self.next_breakpoint_id);
            self.next_breakpoint_id += 1;
            let label = HaltLabel {
                name: probe.name.clone(),
                condition,
                width: probe.width(),
            };
            self.halt_labels.insert(id, label);
            ProbeBreakpoint { id, condition }
        });
    }

    fn reset_watch(&mut self) {
        let set: Vec<BreakpointId> = self
            .probes
            .iter()
            .filter_map(|probe| Some(probe.breakpoint?.id))
            .collect();
        self.halt_labels.retain(|id, _| set.contains(id));
        self.world_breakpoints = Breakpoints::default();
    }

    fn watch_world(&mut self, world: &impl BlockView) {
        let previous = mem::take(&mut self.world_breakpoints);
        let breakpoints = self.probes.iter().filter_map(|probe| {
            let breakpoint = probe.breakpoint?;
            Some(Breakpoint {
                id: breakpoint.id,
                bits: probe.positions.iter().copied().map(Bit::At).collect(),
                condition: breakpoint.condition,
            })
        });
        let powered = |pos| powered_at(world, pos) == Some(true);
        self.world_breakpoints = Breakpoints::new(breakpoints.collect(), Some(&previous), powered);
    }

    fn watch(&self, layout: &Layout, world: &impl BlockView) -> Watch<BlockPos> {
        let breakpoints = self.probes.iter().filter_map(|probe| {
            let breakpoint = probe.breakpoint?;
            Some(Breakpoint {
                id: breakpoint.id,
                bits: probe.bits(layout, world),
                condition: breakpoint.condition,
            })
        });
        let breakpoints = breakpoints.collect();
        Watch { breakpoints }
    }

    fn describe_halt(&self, hit: Hit, tick: u64) -> String {
        match self.halt_labels.get(&hit.breakpoint) {
            Some(label) => format!("Halted at tick {tick}: {}.", label.describe_hit(hit.value)),
            None => format!("Halted at tick {tick}."),
        }
    }
}

impl Plot {
    pub fn probes(&self) -> &[Probe] {
        &self.probes.probes
    }

    pub fn probe(&self, name: &str) -> Option<&Probe> {
        self.probes.get(name)
    }

    pub fn add_probe(&mut self, probe: Probe) -> Result<(), ProbeError> {
        self.probes.add(probe)?;
        self.change_probes();
        Ok(())
    }

    pub fn remove_probe(&mut self, name: &str) -> Result<(), ProbeError> {
        self.probes.remove(name)?;
        self.change_probes();
        Ok(())
    }

    pub fn clear_probes(&mut self) -> usize {
        let count = self.probes.clear();
        self.change_probes();
        count
    }

    pub fn set_breakpoint(&mut self, name: &str, condition: Option<Condition>) {
        self.probes.set_breakpoint(name, condition);
        self.update_watch();
    }

    pub fn halts_on_breakpoints(&self) -> bool {
        self.simulation.halts_on_breakpoints()
    }

    pub(super) fn probe_statuses(&self) -> Vec<ProbeStatus> {
        self.probes
            .iter()
            .map(|probe| ProbeStatus {
                name: probe.name.clone(),
                value: probe.read(&self.world).map(|value| probe.format(value)),
            })
            .collect()
    }

    fn change_probes(&mut self) {
        self.update_watch();
        self.refresh_scoreboard();
    }

    pub(super) fn restart_watch(&mut self) {
        self.probes.reset_watch();
        self.update_watch();
    }

    pub(super) fn update_watch(&mut self) {
        self.probes.watch_world(&self.world);
        let PlotSimulation::Compiled(compiled) = &mut self.simulation else {
            return;
        };
        compiled.watch(self.probes.watch(compiled.layout(), &self.world));
    }

    pub(super) fn report_halt(&mut self, hit: Hit, tick: u64) {
        let message = self.probes.describe_halt(hit, tick);
        self.broadcast(&message);
        self.refresh_scoreboard();
    }
}
