//! Masks as in WorldEdit: <https://worldedit.enginehub.org/en/latest/usage/general/masks/>

use mchprs_blocks::block_state::{block_state, state_id, state_list, StateList};
use mchprs_blocks::block_store::BlockView;
use mchprs_blocks::blocks::Block;
use mchprs_blocks::{BlockFace, BlockPos};
use mchprs_syntax::expect::{quotable, Expect};
use mchprs_syntax::reader::{Reader, SyntaxError};
use rand::RngExt;

const NAMED_MASKS: &[(&str, WorldEditMask)] = &[
    ("#existing", WorldEditMask::Existing),
    ("#solid", WorldEditMask::Solid),
    ("#surface", WorldEditMask::Surface),
    ("#exposed", WorldEditMask::Surface),
];

#[derive(Clone, Debug)]
pub enum WorldEditMask {
    All(Vec<WorldEditMask>),
    Not(Box<WorldEditMask>),
    Offset { y: i32, mask: Box<WorldEditMask> },
    Noise(f32),
    States { lenient: bool, states: StateList },
    Existing,
    Solid,
    Surface,
    Blocks(Vec<BlockMatcher>),
}

impl WorldEditMask {
    pub fn parse(reader: &mut Reader<'_>, expect: &mut Expect) -> Result<Self, SyntaxError> {
        quotable(reader, expect, |reader, expect, quoted| {
            if !quoted {
                return component(reader, expect);
            }
            reader.take_while(char::is_whitespace);
            let mut components = vec![component(reader, expect)?];
            loop {
                let separated = !reader.take_while(char::is_whitespace).is_empty();
                if !separated || (!reader.can_read() && !expect.is_suggesting()) {
                    break;
                }
                components.push(component(reader, expect)?);
            }
            Ok(match components.len() {
                1 => components.swap_remove(0),
                _ => Self::All(components),
            })
        })
    }

    pub fn matches(&self, world: &impl BlockView, pos: BlockPos) -> bool {
        match self {
            Self::All(masks) => masks.iter().all(|mask| mask.matches(world, pos)),
            Self::Not(mask) => !mask.matches(world, pos),
            Self::Offset { y, mask } => mask.matches(world, BlockPos::new(pos.x, pos.y + y, pos.z)),
            Self::Noise(chance) => rand::rng().random::<f32>() < *chance,
            Self::States { lenient, states } => states.matches(world.block(pos), *lenient),
            Self::Existing => world.block(pos) != Block::Air,
            Self::Solid => is_movement_blocker(world.block(pos)),
            Self::Surface => {
                world.block(pos) != Block::Air
                    && BlockFace::values()
                        .into_iter()
                        .any(|face| world.block(pos.offset(face)) == Block::Air)
            }
            Self::Blocks(matchers) => {
                let block = world.block(pos);
                matchers.iter().any(|matcher| matcher.matches(block))
            }
        }
    }
}

#[derive(Clone, Debug)]
pub enum BlockMatcher {
    Type {
        name: &'static str,
        states: StateList,
    },
    State(Block),
}

impl BlockMatcher {
    fn matches(&self, block: Block) -> bool {
        match self {
            Self::Type { name, states } => {
                block.get_name() == *name && states.matches(block, false)
            }
            Self::State(expected) => *expected == block,
        }
    }
}

fn component(reader: &mut Reader<'_>, expect: &mut Expect) -> Result<WorldEditMask, SyntaxError> {
    let mut negated = false;
    let mut offset = 0;
    loop {
        match reader.peek() {
            Some('!') => negated = !negated,
            Some('>') => offset -= 1,
            Some('<') => offset += 1,
            _ => break,
        }
        reader.read();
    }
    let mut mask = predicate(reader, expect)?;
    if offset != 0 {
        mask = WorldEditMask::Offset {
            y: offset,
            mask: Box::new(mask),
        };
    }
    if negated {
        mask = WorldEditMask::Not(Box::new(mask));
    }
    Ok(mask)
}

fn predicate(reader: &mut Reader<'_>, expect: &mut Expect) -> Result<WorldEditMask, SyntaxError> {
    let start = reader.cursor();
    match reader.peek() {
        Some('%') => {
            reader.read();
            let text = reader.take_token();
            text.parse::<f32>()
                .ok()
                .filter(|percent| (0.0..=100.0).contains(percent))
                .map(|percent| WorldEditMask::Noise(percent / 100.0))
                .ok_or_else(|| expect.fail(start, format!("Invalid percentage: %{text}")))
        }
        Some('^') => {
            reader.read();
            let lenient = reader.peek() != Some('=');
            if !lenient {
                reader.read();
            }
            if reader.peek() != Some('[') {
                return Err(expect.fail(reader.cursor(), "Expected [state=value] after ^"));
            }
            let states = state_list(reader, expect, None)?;
            Ok(WorldEditMask::States { lenient, states })
        }
        Some('#') => {
            let name = reader.take_while(|ch| ch == '#' || ch.is_ascii_alphanumeric());
            expect.offer(reader, start, || NAMED_MASKS.iter().map(|(name, _)| *name));
            NAMED_MASKS
                .iter()
                .find(|(candidate, _)| *candidate == name)
                .map(|(_, mask)| mask.clone())
                .ok_or_else(|| expect.fail(start, format!("Unknown mask: {name}")))
        }
        Some('$') => Err(expect.fail(start, "Biome masks aren't supported")),
        _ => {
            expect.offer(reader, start, || NAMED_MASKS.iter().map(|(name, _)| *name));
            let mut matchers = vec![block_matcher(reader, expect)?];
            while reader.peek() == Some(',') {
                reader.read();
                matchers.push(block_matcher(reader, expect)?);
            }
            Ok(WorldEditMask::Blocks(matchers))
        }
    }
}

fn block_matcher(
    reader: &mut Reader<'_>,
    expect: &mut Expect,
) -> Result<BlockMatcher, SyntaxError> {
    if reader.peek() == Some('=') {
        reader.read();
        return state_id(reader, expect).map(BlockMatcher::State);
    }
    let (block, states) = block_state(reader, expect)?;
    Ok(BlockMatcher::Type {
        name: block.get_name(),
        states,
    })
}

// Vanilla 1.20.4's blocksMotion, which WorldEdit's #solid tests and the block data lacks.
fn is_movement_blocker(block: Block) -> bool {
    !matches!(
        block,
        Block::Air
            | Block::RedstoneWire(_)
            | Block::Lever { .. }
            | Block::RedstoneTorch { .. }
            | Block::RedstoneWallTorch { .. }
            | Block::Repeater(_)
            | Block::TripwireHook { .. }
            | Block::Comparator(_)
            | Block::SeaPickle { .. }
    ) && block.button().is_none()
}
