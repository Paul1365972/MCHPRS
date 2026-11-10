use super::PlotSimulation;
use crate::plot::world::PlotWorld;
use crate::plot::Plot;
use mchprs_blocks::BlockPos;
use mchprs_simulation::component::powered_at;
use mchprs_simulation::schedule::{Chunk, Schedule};
use mchprs_simulation::ticks::{run_until_hit, settle_until_hit, Ran};
use mchprs_simulation::watch::{Breakpoints, Hit};
use std::convert::Infallible;
use std::time::Instant;

struct WorldTicks<'a> {
    world: &'a mut PlotWorld,
    tick: u64,
    breakpoints: &'a mut Breakpoints<BlockPos>,
}

impl WorldTicks<'_> {
    fn tick(&mut self) -> Option<Hit> {
        self.world.tick();
        self.tick += 1;
        let world = &*self.world;
        let powered = |pos| powered_at(world, pos) == Some(true);
        self.breakpoints.fire_all(self.tick, powered)
    }

    fn is_settled(&self) -> bool {
        self.world.is_settled()
    }

    fn run(&mut self, chunk: Chunk) -> Result<Ran, Infallible> {
        Ok(match chunk {
            Chunk::Run(ticks) => run_until_hit(self, ticks, WorldTicks::tick),
            Chunk::Settle(limit) => {
                settle_until_hit(self, limit, WorldTicks::is_settled, WorldTicks::tick)
            }
        })
    }
}

impl Plot {
    pub(super) fn run_world_ticks(&mut self) {
        self.update_entity_boxes();
        let PlotSimulation::World(schedule) = &mut self.simulation else {
            return;
        };
        let mut ticks = WorldTicks {
            world: &mut self.world,
            tick: self.ticks,
            breakpoints: self.probes.world_breakpoints(),
        };
        let mut events = Vec::new();
        let Ok((ran, _)) = schedule.run(Instant::now(), &mut events, |chunk| ticks.run(chunk));
        self.record_ticks(ran);
        self.handle_events(&events, 0);
        if !events.is_empty() {
            self.show_world();
        }
    }

    pub(super) fn run_world(&mut self) {
        self.simulation = PlotSimulation::World(Schedule::default());
        self.update_tick_rate();
        self.restart_watch();
        self.publish_world();
        self.refresh_scoreboard();
    }
}
