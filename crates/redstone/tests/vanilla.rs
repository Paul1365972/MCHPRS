use mchprs_blocks::block_entities::BlockEntity;
use mchprs_blocks::block_state;
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::block_volume::BlockVolume;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::region::Region;
use mchprs_blocks::BlockPos;
use mchprs_redstone::interpreter::Interpreter;
use std::fs;
use std::path::{Path, PathBuf};

const GAME_TICKS_PER_REDSTONE_TICK: u32 = 2;
const MARGIN: i32 = 2;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Observed {
    block: Block,
    comparator_output: Option<u16>,
}

impl Observed {
    const AIR: Observed = Observed {
        block: Block::Air,
        comparator_output: None,
    };

    fn parse(state: &str) -> Observed {
        let (state, comparator_output) = match state.split_once("{OutputSignal:") {
            Some((state, output)) => {
                let output = output.trim_end_matches('}');
                (state, Some(output.parse().expect("an output signal")))
            }
            None => (state, None),
        };
        let block = block_state::stored(state)
            .ok()
            .flatten()
            .unwrap_or_else(|| panic!("unknown block state {state}"));
        Observed {
            block: without_hopper_lock(block),
            comparator_output,
        }
    }

    fn at(world: &Interpreter<BlockVolume>, pos: BlockPos) -> Observed {
        let comparator_output = match world.block_entity(pos) {
            Some(BlockEntity::Comparator { output_strength }) => Some(u16::from(*output_strength)),
            _ => None,
        };
        Observed {
            block: without_hopper_lock(world.block(pos)),
            comparator_output,
        }
    }
}

fn without_hopper_lock(block: Block) -> Block {
    match block {
        Block::Hopper { facing, .. } => Block::Hopper {
            enabled: true,
            facing,
        },
        block => block,
    }
}

struct Replay {
    origin: BlockPos,
    observed: Region,
    states: Vec<Observed>,
    vanilla: Vec<Observed>,
    world: Interpreter<BlockVolume>,
    game_tick: u32,
}

impl Replay {
    fn parse_header<'a>(lines: &mut impl Iterator<Item = &'a str>) -> Replay {
        let mut header = |expected: &str| {
            let (keyword, operands) = words(lines.next().expect("a header line"));
            assert_eq!(keyword, expected);
            coords(&operands)
        };
        let origin = header("origin");
        let size = header("size");
        let mut states = Vec::new();
        let start = loop {
            match words(lines.next().expect("a header line")) {
                ("state", operands) => {
                    assert_eq!(operands[0].parse(), Ok(states.len()));
                    states.push(Observed::parse(operands[1]));
                }
                ("start", operands) => break operands,
                (keyword, _) => panic!("unexpected {keyword} in the header"),
            }
        };
        let observed = Region::new(origin, origin + size - BlockPos::new(1, 1, 1));
        let margin = BlockPos::new(MARGIN, MARGIN, MARGIN);
        let bounds = Region::new(observed.min() - margin, observed.max() + margin);
        let mut replay = Replay {
            origin,
            observed,
            states,
            vanilla: vec![Observed::AIR; observed.positions().count()],
            world: Interpreter::new(BlockVolume::empty(bounds), Vec::new()),
            game_tick: 0,
        };
        replay.record(&start);
        replay.start();
        replay
    }

    fn pos(&self, relative: &str) -> BlockPos {
        let mut coords = relative
            .split(',')
            .map(|coord| coord.parse::<i32>().expect("a coordinate"));
        let mut coord = || coords.next().expect("three coordinates");
        self.origin + BlockPos::new(coord(), coord(), coord())
    }

    fn index(&self, pos: BlockPos) -> usize {
        self.observed
            .positions()
            .position(|observed| observed == pos)
            .expect("a position in the observed region")
    }

    fn record(&mut self, changes: &[&str]) {
        for change in changes {
            let (pos, state) = change.split_once('=').expect("a change");
            let index = self.index(self.pos(pos));
            self.vanilla[index] = self.states[state.parse::<usize>().expect("a state id")];
        }
    }

    fn start(&mut self) {
        for (pos, observed) in self.observed.positions().zip(&self.vanilla) {
            self.world.set_block(pos, observed.block);
            if let Some(output) = observed.comparator_output {
                let output_strength = u8::try_from(output).expect("a comparator output fitting u8");
                let comparator = BlockEntity::Comparator { output_strength };
                self.world.set_block_entity(pos, Some(comparator));
            }
        }
    }

    fn difference(&self, moment: &str) -> Option<String> {
        let mut positions = self.observed.positions().zip(&self.vanilla);
        positions.find_map(|(pos, &vanilla)| {
            let interpreter = Observed::at(&self.world, pos);
            (interpreter != vanilla).then(|| {
                format!(
                    "{moment} game tick {} at {pos}: vanilla has {vanilla:?}, the interpreter has {interpreter:?}",
                    self.game_tick
                )
            })
        })
    }

    fn wait(&mut self, game_ticks: u32, changes: &[&str]) -> Option<String> {
        let redstone_ticks = game_ticks / GAME_TICKS_PER_REDSTONE_TICK;
        for redstone_tick in 1..=redstone_ticks {
            self.world.tick();
            self.game_tick += GAME_TICKS_PER_REDSTONE_TICK;
            if redstone_tick == redstone_ticks {
                self.record(changes);
            }
            if let Some(difference) = self.difference("after") {
                return Some(difference);
            }
        }
        None
    }

    fn act(
        &mut self,
        operands: &[&str],
        act: fn(&mut Interpreter<BlockVolume>, BlockPos),
        moment: &str,
    ) -> Option<String> {
        let (changes, positions): (Vec<&str>, Vec<&str>) =
            operands.iter().partition(|operand| operand.contains('='));
        for pos in positions {
            let pos = self.pos(pos);
            act(&mut self.world, pos);
        }
        self.record(&changes);
        self.difference(moment)
    }
}

fn words(line: &str) -> (&str, Vec<&str>) {
    let mut words = line.split(' ');
    let keyword = words.next().expect("a keyword");
    (keyword, words.collect())
}

fn coords(operands: &[&str]) -> BlockPos {
    let coord = |index: usize| operands[index].parse::<i32>().expect("a coordinate");
    BlockPos::new(coord(0), coord(1), coord(2))
}

fn first_difference(trace: &str) -> Option<String> {
    let mut lines = trace.lines();
    let mut replay = Replay::parse_header(&mut lines);
    for line in lines {
        let (keyword, operands) = words(line);
        let difference = match keyword {
            "wait" => {
                let game_ticks = operands[0].parse().expect("a number of game ticks");
                replay.wait(game_ticks, &operands[1..])
            }
            "use" => replay.act(
                &operands,
                Interpreter::use_block,
                "after the uses following",
            ),
            _ => panic!("unknown trace line: {line}"),
        };
        if difference.is_some() {
            return difference;
        }
    }
    None
}

fn traces(directory: &Path) -> Vec<(PathBuf, String)> {
    let mut paths: Vec<PathBuf> = fs::read_dir(directory)
        .unwrap_or_else(|error| panic!("cannot read {}: {error}", directory.display()))
        .map(|entry| entry.unwrap().path())
        .collect();
    paths.retain(|path| {
        path.extension()
            .is_some_and(|extension| extension == "trace")
    });
    paths.sort();
    paths
        .into_iter()
        .map(|path| {
            let trace = fs::read_to_string(&path).unwrap();
            (path, trace)
        })
        .collect()
}

#[test]
fn interpreter_follows_the_traces_recorded_from_vanilla() {
    let directory = Path::new("tests/vanilla");
    let agreeing = traces(directory);
    assert!(!agreeing.is_empty(), "no traces in {}", directory.display());
    let mut failures = Vec::new();
    for (path, trace) in agreeing {
        if let Some(difference) = first_difference(&trace) {
            failures.push(format!("{} differs {difference}", path.display()));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
