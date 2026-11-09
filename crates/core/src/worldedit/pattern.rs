//! Patterns as in WorldEdit: <https://worldedit.enginehub.org/en/latest/usage/general/patterns/>

use super::replace_block;
use mchprs_blocks::block_state::{block_state, state_id, state_list, StateList};
use mchprs_blocks::block_store::{BlockStore, BlockView};
use mchprs_blocks::blocks::Block;
use mchprs_blocks::BlockPos;
use mchprs_syntax::expect::{quotable, Expect};
use mchprs_syntax::reader::{Reader, SyntaxError};
use rand::RngExt;
use std::collections::HashMap;

#[derive(Clone, Debug)]
pub enum WorldEditPattern {
    Single(BlockPattern),
    Random {
        patterns: Vec<WeightedPattern>,
        weight_total: f32,
    },
}

impl WorldEditPattern {
    pub fn parse(reader: &mut Reader<'_>, expect: &mut Expect) -> Result<Self, SyntaxError> {
        quotable(reader, expect, |reader, expect, quoted| {
            let start = reader.cursor();
            let skip_spaces = |reader: &mut Reader<'_>| {
                if quoted {
                    reader.take_while(char::is_whitespace);
                }
            };
            let mut patterns = Vec::new();
            let mut weighted = false;
            loop {
                skip_spaces(reader);
                let weight = weight(reader, expect)?;
                if weight.is_some() {
                    weighted = true;
                    skip_spaces(reader);
                }
                let pattern = block_pattern(reader, expect)?;
                patterns.push(WeightedPattern {
                    weight: weight.unwrap_or(1.0),
                    pattern,
                });
                skip_spaces(reader);
                if reader.peek() != Some(',') {
                    break;
                }
                reader.read();
            }
            if patterns.len() == 1 && !weighted {
                return Ok(Self::Single(patterns.swap_remove(0).pattern));
            }
            let weight_total: f32 = patterns.iter().map(|pattern| pattern.weight).sum();
            if !weight_total.is_finite() {
                return Err(expect.fail(start, "Pattern weight total is too large"));
            }
            if weight_total <= 0.0 {
                return Err(expect.fail(start, "Pattern weights must add up to more than zero"));
            }
            patterns.retain(|pattern| pattern.weight > 0.0);
            Ok(Self::Random {
                patterns,
                weight_total,
            })
        })
    }

    pub fn place(&self, world: &mut impl BlockStore, pos: BlockPos) -> bool {
        let pattern = self.select();
        let block = pattern.pick(world, pos);
        if pattern.keeps_block_entity() {
            world.set_block(pos, block)
        } else {
            replace_block(world, pos, block)
        }
    }

    fn select(&self) -> &BlockPattern {
        match self {
            Self::Single(pattern) => pattern,
            Self::Random {
                patterns,
                weight_total,
            } => {
                let mut random = rand::rng().random_range(0.0..*weight_total);
                let selected = patterns.iter().find(|pattern| {
                    random -= pattern.weight;
                    random < 0.0
                });
                &selected.unwrap_or_else(|| patterns.last().unwrap()).pattern
            }
        }
    }
}

#[derive(Clone, Debug)]
pub struct WeightedPattern {
    weight: f32,
    pattern: BlockPattern,
}

#[derive(Clone, Debug)]
pub enum BlockPattern {
    Fixed(Block),
    TypeApplying {
        block: Option<Block>,
        states: StateList,
    },
}

impl BlockPattern {
    fn pick(&self, world: &impl BlockView, pos: BlockPos) -> Block {
        match self {
            Self::Fixed(block) => *block,
            Self::TypeApplying { block, states } => {
                let existing = world.block(pos);
                let mut block = block.unwrap_or(existing);
                let existing_properties = existing.properties();
                let mut properties: HashMap<&str, &str> = existing_properties
                    .iter()
                    .map(|(name, value)| (*name, value.as_str()))
                    .collect();
                for (name, value) in states.states() {
                    properties.insert(name, value);
                }
                properties.retain(|name, value| {
                    block
                        .property_values(name)
                        .is_some_and(|values| values.contains(value))
                });
                block.set_properties(properties);
                block
            }
        }
    }

    fn keeps_block_entity(&self) -> bool {
        matches!(self, Self::TypeApplying { block: None, .. })
    }
}

fn weight(reader: &mut Reader<'_>, expect: &mut Expect) -> Result<Option<f32>, SyntaxError> {
    let start = reader.cursor();
    let mut ahead = *reader;
    let text = ahead.take_while(|ch| !matches!(ch, '%' | ',' | '[') && !ch.is_whitespace());
    if ahead.read() != Some('%') {
        return Ok(None);
    }
    let weight = text
        .parse::<f32>()
        .ok()
        .filter(|weight| weight.is_finite() && *weight >= 0.0)
        .ok_or_else(|| expect.fail(start, format!("Invalid pattern weight: {text}")))?;
    let mut rest = ahead;
    let block = rest.take_while(|ch| !matches!(ch, ',' | '[') && !ch.is_whitespace());
    if block.contains('%') {
        return Err(expect.fail(start, format!("Invalid pattern weight: {text}%{block}")));
    }
    *reader = ahead;
    Ok(Some(weight))
}

fn block_pattern(
    reader: &mut Reader<'_>,
    expect: &mut Expect,
) -> Result<BlockPattern, SyntaxError> {
    let start = reader.cursor();
    match reader.peek() {
        Some('^') => {
            reader.read();
            if reader.peek() == Some('[') {
                let states = state_list(reader, expect, None)?;
                return Ok(BlockPattern::TypeApplying {
                    block: None,
                    states,
                });
            }
            let (block, states) = block_state(reader, expect)?;
            Ok(BlockPattern::TypeApplying {
                block: Some(block),
                states,
            })
        }
        Some('=') => {
            reader.read();
            state_id(reader, expect).map(BlockPattern::Fixed)
        }
        Some('*' | '#') => Err(expect.fail(start, "This pattern type isn't supported")),
        _ => {
            let (block, states) = block_state(reader, expect)?;
            Ok(BlockPattern::Fixed(states.applied_to(block)))
        }
    }
}
