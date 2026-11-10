use super::report::{self, caught, illegal_trace, FailedCase, Failure};
use super::rng::Rng;
use super::scenario_generator::{self, moment, random_steps};
use super::world_generator::builder::random_world;
use super::world_generator::{self, describe_block, position, warm_up};
use super::Cases;
use crate::input::World;
use crate::oracle::trace::{NodeView, PendingTicks, Seen, View};
use crate::oracle::{Oracle, Step, Verdict};
use crate::setup::Setup;
use crate::{Error, Outcome};
use anyhow::{anyhow, bail, Context};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::BlockPos;
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::extract::component_at;
use mchprs_redstone::input::Input;
use mchprs_redstone::interpreter::Interpreter;
use mchprs_redstone::sound::{Sound, SoundSource};
use mchprs_redstone::tick_queue::PendingTick;
use mchprs_simulation::readback::Readback;
use mchprs_simulation::simulated_world::SimulatedWorld;
use rustc_hash::FxHashSet;
use std::mem;

const INTERPRETER: &str = "vanilla";
/// `reference` simulates every block, so the world it writes back is whole
const STOPPING_SETUP: &str = "reference";
const RUNNING_SETUPS: [&str; 2] = ["direct -o", "direct -o -i"];
const TICKS: u64 = 80;

type Sounds = Vec<(BlockPos, Sound)>;

struct RunningSetup {
    name: &'static str,
    simulated_world: SimulatedWorld,

    shown_nodes: Vec<NodeId>,
    trace: Vec<Seen>,
    /// Where the state differed after the tick before, the sounds of this tick may differ too
    differed_before: Vec<BlockPos>,
    sounds_since_tick: Sounds,
    stops_after: Option<u64>,
    interpreter_resumed: bool,
    diverged: Option<String>,
}

impl RunningSetup {
    fn compile(name: &'static str, world: &World, circuit: &Circuit) -> Result<Self, Error> {
        let simulated_world = Setup::parse(name)?.simulate(world);
        let mut simulated_world =
            simulated_world.with_context(|| format!("{name} does not compile"))?;
        touch_plates(&mut simulated_world, world, circuit)?;
        let shown_nodes: Vec<NodeId> = circuit
            .ids()
            .filter(|&node| simulated_world.is_current(position(circuit, node)))
            .collect();
        Ok(RunningSetup {
            name,
            trace: vec![Seen {
                steps_played: 0,
                view: view(&simulated_world, circuit, &shown_nodes, &[])?,
            }],
            simulated_world,
            shown_nodes,
            differed_before: Vec::new(),
            sounds_since_tick: Vec::new(),
            stops_after: None,
            interpreter_resumed: false,
            diverged: None,
        })
    }

    fn take_sounds(&mut self) {
        let sounds = self.simulated_world.take_sounds();
        self.sounds_since_tick.extend(sounds);
    }

    fn end_tick(
        &mut self,
        world: &World,
        circuit: &Circuit,
        expected_sounds: &[(BlockPos, Sound)],
        steps_played: usize,
        moment: impl Fn() -> String,
    ) -> Result<(), Error> {
        let sounds = last_sound_of_each_block(mem::take(&mut self.sounds_since_tick));
        let mut shown = view(&self.simulated_world, circuit, &self.shown_nodes, &sounds)?;
        let expected = view(world, circuit, &self.shown_nodes, expected_sounds)?;
        let mut differing = Vec::new();
        let nodes = self.shown_nodes.iter().zip(&shown.nodes);
        for ((&node, shown), expected) in nodes.zip(&expected.nodes) {
            if shown != expected {
                differing.push(position(circuit, node));
            }
        }
        let mut differs = !differing.is_empty();
        if self.interpreter_resumed {
            let pending_ticks = complete_pending_ticks(&mut self.simulated_world, INTERPRETER)?;
            let pending_ticks = node_ticks(circuit, pending_ticks)?;
            differs |= pending_ticks != node_ticks(circuit, world.pending_ticks())?;
            shown.pending_ticks = Some(pending_ticks);
        }
        self.trace.push(Seen {
            steps_played,
            view: shown,
        });
        if differs && self.diverged.is_none() {
            self.diverged = Some(moment());
        }
        let name = self.name;
        let unexplained = |difference: String| {
            let moment = moment();
            anyhow!("{name} differs from {INTERPRETER} {moment} but in no component\n{difference}")
        };
        let is_explained = |pos: &BlockPos| differing.contains(pos);
        for pos in world.blocks().region().positions() {
            let [expected, shown] = [
                describe_block(world, pos),
                describe_block(&self.simulated_world, pos),
            ];
            if self.simulated_world.is_current(pos) && expected != shown && !is_explained(&pos) {
                return Err(unexplained(format!(
                    "{pos}: {INTERPRETER} {expected}, {name} {shown}"
                )));
            }
        }
        let mut sounding: Vec<BlockPos> = expected_sounds
            .iter()
            .chain(&sounds)
            .map(|&(pos, _)| pos)
            .collect();
        sounding.retain(|pos| {
            self.simulated_world.is_current(*pos)
                && !is_explained(pos)
                && !self.differed_before.contains(pos)
        });
        for pos in sounding {
            let [expected, played] = [expected_sounds, &sounds].map(|sounds| {
                let sounds: Vec<_> = sounds.iter().filter(|&&(at, _)| at == pos).collect();
                listed(&sounds, |(_, sound)| {
                    format!("{} at pitch {}", sound.event, sound.pitch)
                })
            });
            if expected != played {
                return Err(unexplained(format!(
                    "sounds at {pos}: {INTERPRETER} [{expected}], {name} [{played}]"
                )));
            }
        }
        self.differed_before = differing;
        Ok(())
    }

    fn stop(mut self, world: &World, circuit: &Circuit) -> Result<RunningSetup, Error> {
        let pending_ticks = sorted(complete_pending_ticks(
            &mut self.simulated_world,
            self.name,
        )?);
        let (blocks, Readback::Complete(write_back)) = self.simulated_world.stop()? else {
            bail!(
                "{} reads back incompletely, so it can't write the world back",
                self.name
            );
        };
        let mut left = Interpreter::new(blocks, Vec::new());
        write_back.apply(&mut left);
        self.simulated_world = Setup::parse(INTERPRETER)?.simulate(&left)?;
        touch_plates(&mut self.simulated_world, world, circuit)?;
        self.interpreter_resumed = true;
        let resumed_ticks = sorted(complete_pending_ticks(
            &mut self.simulated_world,
            INTERPRETER,
        )?);
        if resumed_ticks != pending_ticks {
            let [had, resumed] =
                [pending_ticks, resumed_ticks].map(|ticks| listed(&ticks, pending_tick));
            return Err(anyhow!(
                "{} stopped with the pending ticks [{had}], but wrote back [{resumed}]",
                self.name
            ));
        }
        Ok(self)
    }
}

fn complete_pending_ticks(
    simulated_world: &mut SimulatedWorld,
    name: &str,
) -> Result<Vec<(BlockPos, PendingTick)>, Error> {
    let Readback::Complete(write_back) = simulated_world.read()? else {
        bail!("{name} reads back incompletely, so its pending ticks can't be compared");
    };
    Ok(write_back.ticks)
}

pub fn command(cases: &Cases) -> Outcome {
    report::run(cases, |seed| {
        let mut rng = Rng::new(seed);
        let mut world = random_world(&mut rng);
        let case = caught(|| Ok(Circuit::new(warm_up(&mut world, &mut rng)?)?));
        let described = world_generator::describe(&world);
        let failure = |problem, case| Failure {
            command: "worlds".to_owned(),
            problem,
            case: FailedCase::World(case),
        };
        let circuit = case.map_err(|problem| failure(problem, described.clone()))?;
        let steps = random_steps(&circuit, &mut rng, TICKS);
        let stopped_after = rng.within(1..=TICKS);
        let checked = caught(|| check(world, &circuit, &steps, stopped_after));
        let scenario = scenario_generator::describe(&circuit, &steps);
        checked.map_err(|problem| failure(problem, format!("{described}\n{scenario}")))
    })
}

fn check(
    mut world: World,
    circuit: &Circuit,
    steps: &[Step],
    stopped_after: u64,
) -> Result<Verdict, Error> {
    let all_nodes: Vec<NodeId> = circuit.ids().collect();
    let mut setups = vec![RunningSetup {
        stops_after: Some(stopped_after),
        ..RunningSetup::compile(STOPPING_SETUP, &world, circuit)?
    }];
    for name in RUNNING_SETUPS {
        setups.push(RunningSetup::compile(name, &world, circuit)?);
    }
    let seen = |world: &World, steps_played: usize, sounds_of_tick: &[(BlockPos, Sound)]| {
        Ok::<_, Error>(Seen {
            steps_played,
            view: View {
                pending_ticks: Some(node_ticks(circuit, world.pending_ticks())?),
                ..view(world, circuit, &all_nodes, sounds_of_tick)?
            },
        })
    };
    let mut interpreter_trace = vec![seen(&world, 0, &[])?];
    let mut sounds_since_tick = Vec::new();
    let mut tick = 0;
    for (index, &step) in steps.iter().enumerate() {
        let steps_played = index + 1;
        match step {
            Step::Drive(node, input) => {
                let pos = position(circuit, node);
                world
                    .drive(pos, input)
                    .map_err(|error| anyhow!("the block at {pos} {error}"))?;
                sounds_since_tick.extend(world.take_sounds());
                interpreter_trace.push(seen(&world, steps_played, &[])?);
                for setup in &mut setups {
                    setup.simulated_world.drive(pos, input)?;
                    setup.take_sounds();
                }
            }
            Step::Tick => {
                tick += 1;
                world.tick();
                sounds_since_tick.extend(world.take_sounds());
                let sounds = last_sound_of_each_block(mem::take(&mut sounds_since_tick));
                interpreter_trace.push(seen(&world, steps_played, &sounds)?);
                let moment = || moment(circuit, &steps[..=index]);
                let ticked = setups.into_iter().map(|mut setup| {
                    setup.simulated_world.run(1)?;
                    setup.take_sounds();
                    setup.end_tick(&world, circuit, &sounds, steps_played, moment)?;
                    match setup.stops_after == Some(tick) {
                        true => setup.stop(&world, circuit),
                        false => Ok(setup),
                    }
                });
                setups = ticked.collect::<Result<_, Error>>()?;
            }
        }
    }
    let judge = |observed: &[NodeId], trace: &[Seen], seen_by: &str| {
        let verdict = Oracle::new(circuit, observed).check(steps, trace);
        verdict.map_err(|illegal| illegal_trace(circuit, observed, steps, seen_by, illegal))
    };
    let mut verdict = judge(&all_nodes, &interpreter_trace, INTERPRETER)?;
    for setup in &setups {
        let Some(diverged) = &setup.diverged else {
            continue;
        };
        let name = setup.name;
        let differs =
            |illegal| anyhow!("{name} first differs from {INTERPRETER} {diverged}\n{illegal}");
        if let Verdict::Skipped = judge(&setup.shown_nodes, &setup.trace, name).map_err(differs)? {
            verdict = Verdict::Skipped;
        }
    }
    Ok(verdict)
}

/// A pressure plate is untouched after compiling (docs/Library.md, Rules)
fn touch_plates(
    simulated_world: &mut SimulatedWorld,
    world: &World,
    circuit: &Circuit,
) -> Result<(), Error> {
    for node in circuit.ids() {
        let pos = position(circuit, node);
        if world.is_standing_on(pos) {
            simulated_world.drive(pos, Input::Touch(true))?;
        }
    }
    simulated_world.take_sounds();
    Ok(())
}

fn view(
    blocks: &impl BlockView,
    circuit: &Circuit,
    nodes: &[NodeId],
    sounds: &[(BlockPos, Sound)],
) -> Result<View, Error> {
    let mut shown = Vec::new();
    for &node in nodes {
        let pos = position(circuit, node);
        let component = component_at(blocks, pos)?;
        let (_, state) = component.ok_or_else(|| anyhow!("no component is left at {pos}"))?;
        let is_note_here =
            |&(at, sound): &(BlockPos, Sound)| at == pos && sound.source == SoundSource::Records;
        shown.push(NodeView {
            state,
            played_note: sounds.iter().any(is_note_here),
        });
    }
    Ok(View {
        nodes: shown,
        pending_ticks: None,
    })
}

fn node_ticks(
    circuit: &Circuit,
    pending_ticks: Vec<(BlockPos, PendingTick)>,
) -> Result<PendingTicks, Error> {
    let node_at = |pos: BlockPos| {
        let node = circuit.ids().find(|&node| position(circuit, node) == pos);
        node.ok_or_else(|| anyhow!("a tick is pending at {pos}, which holds no component"))
    };
    let node_ticks = pending_ticks
        .into_iter()
        .map(|(pos, tick)| Ok((node_at(pos)?, tick)));
    Ok(PendingTicks::new(node_ticks.collect::<Result<_, Error>>()?))
}

fn last_sound_of_each_block(sounds: Sounds) -> Sounds {
    let mut sounding = FxHashSet::default();
    let later_first = sounds.into_iter().rev();
    let mut last_sounds: Sounds = later_first
        .filter(|&(pos, _)| sounding.insert(pos))
        .collect();
    last_sounds.reverse();
    last_sounds
}

fn sorted(mut pending_ticks: Vec<(BlockPos, PendingTick)>) -> Vec<(BlockPos, PendingTick)> {
    pending_ticks.sort_unstable();
    pending_ticks
}

fn pending_tick((pos, tick): &(BlockPos, PendingTick)) -> String {
    format!("{pos} in {tick}")
}

fn listed<T>(items: &[T], describe: impl Fn(&T) -> String) -> String {
    let described: Vec<String> = items.iter().map(describe).collect();
    described.join(", ")
}
