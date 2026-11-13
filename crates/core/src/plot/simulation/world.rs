use super::PlotSimulation;
use crate::plot::world::PlotWorld;
use crate::plot::Plot;
use mchprs_simulation::schedule::{Chunk, Schedule};
use mchprs_simulation::ticks::{Ran, Stop};
use std::convert::Infallible;
use std::time::Instant;

struct WorldTicks<'a> {
    world: &'a mut PlotWorld,
}

impl WorldTicks<'_> {
    fn is_settled(&self) -> bool {
        self.world.is_settled()
    }

    fn run(&mut self, chunk: Chunk) -> Result<Ran, Infallible> {
        Ok(match chunk {
            Chunk::Run(ticks) => {
                for _ in 0..ticks {
                    self.world.tick();
                }
                Ran { ticks, stop: None }
            }
            Chunk::Settle(limit) => {
                let mut ticks = 0;
                while !self.is_settled() && ticks < limit {
                    self.world.tick();
                    ticks += 1;
                }
                Ran {
                    ticks,
                    stop: self.is_settled().then_some(Stop::Settled),
                }
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
        };
        let mut events = Vec::new();
        let Ok((ran, _)) = schedule.run(Instant::now(), &mut events, |chunk| ticks.run(chunk));
        self.record_ticks(ran);
        self.handle_events(&events);
        if !events.is_empty() {
            self.show_world();
        }
    }

    pub(super) fn run_world(&mut self) {
        self.simulation = PlotSimulation::World(Schedule::default());
        self.update_tick_rate();
        self.publish_world();
        self.refresh_scoreboard();
    }
}
