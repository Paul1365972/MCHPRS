use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_redstone::input::Input;
use mchprs_redstone::tick_queue::{PendingTick, TickPriority};
use mchprs_simulation::block_changes::BlockChanges;
use mchprs_simulation::input::DriveError;
use mchprs_simulation::layout::Layout;
use mchprs_simulation::readback::Readback;
use mchprs_simulation::simulated_world::SimulatedWorld;
use mchprs_simulation::stepper;
use mchprs_simulation::ticks::{Ran, SimulationError, Stop};
use mchprs_simulation::watch::{BreakpointId, Hit, Watch};
use mchprs_simulation::write_back::WriteBack;
use mchprs_simulation::{
    Blocks, Capabilities, Command, End, Event, Run, Running, Simulation, Update, View,
};
use std::sync::{Arc, Mutex};
use std::thread::{self, ThreadId};
use std::time::Duration;

const HALTING_TICK: u64 = 10;

#[derive(Default)]
struct Counter {
    ticks: u64,
    halting_tick: Option<u64>,
    driven_at: Arc<Mutex<Vec<u64>>>,
    ran_on: Arc<Mutex<Vec<ThreadId>>>,
    unshown: Vec<BlockPos>,
    pending_ticks: Vec<(BlockPos, PendingTick)>,
}

impl Counter {
    fn write_back(&self) -> WriteBack {
        WriteBack {
            changes: BlockChanges::default(),
            ticks: self.pending_ticks.clone(),
            region: Region::new(BlockPos::zero(), BlockPos::zero()),
        }
    }
}

impl Simulation<Blocks> for Counter {
    fn capabilities(&self) -> Capabilities {
        Capabilities {
            breakpoints: true,
            settling: true,
            write_back: true,
        }
    }

    fn ticks(&self) -> u64 {
        self.ticks
    }

    fn run(&mut self, ticks: u64) -> Result<Ran, SimulationError> {
        self.ran_on.lock().unwrap().push(thread::current().id());
        let end = self.ticks + ticks;
        if let Some(tick) = self
            .halting_tick
            .filter(|&tick| (self.ticks + 1..=end).contains(&tick))
        {
            let ran = tick - self.ticks;
            self.ticks = tick;
            let hit = Hit {
                breakpoint: BreakpointId(0),
                tick,
                value: 1,
            };
            return Ok(Ran {
                ticks: ran,
                stop: Some(Stop::Breakpoint(hit)),
            });
        }
        self.ticks = end;
        Ok(Ran { ticks, stop: None })
    }

    fn settle(&mut self, _: u64) -> Result<Ran, SimulationError> {
        Ok(Ran {
            ticks: 0,
            stop: Some(Stop::Settled),
        })
    }

    fn drive(&mut self, pos: BlockPos, _: Input) -> Result<(), DriveError> {
        self.driven_at.lock().unwrap().push(self.ticks);
        self.unshown.push(pos);
        Ok(())
    }

    fn watch(&mut self, _: Watch<BlockPos>) -> Result<(), SimulationError> {
        self.halting_tick = Some(HALTING_TICK);
        Ok(())
    }

    fn show(&mut self, _: View, changes: &mut BlockChanges) -> Result<(), SimulationError> {
        let shown = self
            .unshown
            .drain(..)
            .map(|pos| (pos, Block::RedstoneBlock));
        changes.blocks.extend(shown);
        Ok(())
    }

    fn read(&mut self) -> Result<Readback<WriteBack>, SimulationError> {
        Ok(Readback::Complete(self.write_back()))
    }

    fn stop(mut self: Box<Self>) -> Result<End<Blocks>, SimulationError> {
        let mut changes = BlockChanges::default();
        self.show(View::Observed, &mut changes)?;
        Ok(End {
            changes,
            state: Readback::Complete(self.write_back()),
        })
    }

    fn start(self: Box<Self>) -> Result<Box<dyn Running<Blocks>>, SimulationError> {
        stepper::start(self, "counter")
    }
}

fn start(counter: Counter) -> Box<dyn Running<Blocks>> {
    Box::new(counter).start().unwrap()
}

fn wait_for(
    simulation: &mut dyn Running<Blocks>,
    events: &mut Vec<Event<BlockPos>>,
    wanted: Event<BlockPos>,
) -> Update<Blocks> {
    loop {
        let update = simulation.wait().unwrap();
        events.extend(&update.events);
        if update.events.contains(&wanted) {
            return update;
        }
    }
}

fn shown(simulation: &mut dyn Running<Blocks>) -> Update<Blocks> {
    simulation.send(Command::Show(View::Observed));
    wait_for(simulation, &mut Vec::new(), Event::Shown(View::Observed))
}

#[test]
fn a_drive_and_a_run_of_n_ticks_run_n_ticks_after_the_input() {
    let driven_at = Arc::new(Mutex::new(Vec::new()));
    let counter = Counter {
        driven_at: Arc::clone(&driven_at),
        ..Counter::default()
    };
    let mut simulation = start(counter);
    let pos = BlockPos::new(0, 0, 0);
    for ticks in [3, 4] {
        simulation.send(Command::Drive(pos, Input::Toggle));
        simulation.send(Command::Run(Run::Ticks(ticks)));
        wait_for(&mut *simulation, &mut Vec::new(), Event::RunFinished);
    }
    assert_eq!(*driven_at.lock().unwrap(), [0, 3]);
    assert_eq!(shown(&mut *simulation).ticks, 7);
}

#[test]
fn a_landed_input_shows_itself() {
    let mut simulation = start(Counter::default());
    let pos = BlockPos::new(1, 2, 3);
    simulation.send(Command::Drive(pos, Input::Toggle));
    let landed = simulation.wait().unwrap();
    assert_eq!(landed.changes.blocks, [(pos, Block::RedstoneBlock)]);
    assert!(landed.events.is_empty());
}

#[test]
fn runs_with_a_goal_end_in_the_order_sent() {
    let mut simulation = start(Counter::default());
    simulation.send(Command::Run(Run::Ticks(u64::MAX)));
    simulation.send(Command::Run(Run::Ticks(3)));
    let mut events = Vec::new();
    while events.len() < 2 {
        let update = simulation.wait().unwrap();
        let ends = update.events.into_iter();
        events.extend(
            ends.filter(|event| matches!(event, Event::RunFinished | Event::RunInterrupted)),
        );
    }
    assert!(
        events == [Event::RunInterrupted, Event::RunFinished]
            || events == [Event::RunFinished, Event::RunFinished],
        "{events:?}"
    );
}

#[test]
fn a_breakpoint_leaves_the_simulation_paused() {
    let mut simulation = start(Counter::default());
    simulation.send(Command::Watch(Watch {
        breakpoints: Vec::new(),
    }));
    simulation.send(Command::Run(Run::Unlimited));
    let mut events = Vec::new();
    let halted = wait_for(
        &mut *simulation,
        &mut events,
        Event::Halted(Hit {
            breakpoint: BreakpointId(0),
            tick: HALTING_TICK,
            value: 1,
        }),
    );
    assert_eq!(halted.ticks, HALTING_TICK);
    thread::sleep(Duration::from_millis(50));
    assert_eq!(shown(&mut *simulation).ticks, HALTING_TICK);
}

#[test]
fn stopping_a_paused_simulation_returns_its_pending_ticks_without_running_a_tick() {
    let pending_ticks = vec![
        (
            BlockPos::new(1, 2, 3),
            PendingTick {
                delay: 2,
                priority: TickPriority::High,
            },
        ),
        (
            BlockPos::new(4, 2, 3),
            PendingTick {
                delay: 4,
                priority: TickPriority::Normal,
            },
        ),
    ];
    let counter = Counter {
        pending_ticks: pending_ticks.clone(),
        ..Counter::default()
    };
    let mut simulation = start(counter);
    assert_eq!(shown(&mut *simulation).ticks, 0);
    let Readback::Complete(write_back) = simulation.stop().unwrap().state else {
        panic!("the counter reads back completely");
    };
    assert_eq!(write_back.ticks, pending_ticks);
}

#[test]
fn a_stepped_simulation_runs_on_the_callers_thread() {
    let ran_on = Arc::new(Mutex::new(Vec::new()));
    let counter = Counter {
        ran_on: Arc::clone(&ran_on),
        ..Counter::default()
    };
    let region = Region::new(BlockPos::zero(), BlockPos::new(1, 1, 1));
    let world = BlockVolume::empty(region);
    let mut world = SimulatedWorld::new(world, Box::new(counter), Layout::default());
    let pos = BlockPos::new(1, 0, 1);
    world.drive(pos, Input::Toggle).unwrap();
    assert_eq!(world.run(4).unwrap().ticks, 4);
    assert_eq!(*ran_on.lock().unwrap(), [thread::current().id()]);
    let (blocks, _) = world.stop().unwrap();
    assert_eq!(blocks.block(pos), Block::RedstoneBlock);
}

#[test]
fn starting_keeps_the_state_the_watches_and_the_tick_count() {
    let mut counter = Box::new(Counter::default());
    let pos = BlockPos::new(2, 0, 0);
    counter.run(4).unwrap();
    counter.drive(pos, Input::Toggle).unwrap();
    counter
        .watch(Watch {
            breakpoints: Vec::new(),
        })
        .unwrap();
    let mut simulation = counter.start().unwrap();
    let paused = shown(&mut *simulation);
    assert_eq!(paused.ticks, 4);
    assert_eq!(paused.changes.blocks, [(pos, Block::RedstoneBlock)]);
    simulation.send(Command::Run(Run::Unlimited));
    let mut events = Vec::new();
    let hit = Hit {
        breakpoint: BreakpointId(0),
        tick: HALTING_TICK,
        value: 1,
    };
    let halted = wait_for(&mut *simulation, &mut events, Event::Halted(hit));
    assert_eq!(halted.ticks, HALTING_TICK);
}
