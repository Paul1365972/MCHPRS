use mchprs_blocks::blocks::Block;
use mchprs_blocks::signal_strength::SignalStrength;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Input {
    Set(SignalStrength),
    Toggle,
    Press,
    Touch(bool),
}

impl Input {
    pub fn all() -> impl Iterator<Item = Input> {
        [
            Input::Toggle,
            Input::Press,
            Input::Touch(false),
            Input::Touch(true),
        ]
        .into_iter()
        .chain(SignalStrength::all().map(Input::Set))
    }

    pub fn of_use(block: Block) -> Option<Input> {
        match block {
            Block::Lever { .. } => Some(Input::Toggle),
            block if block.button().is_some() => Some(Input::Press),
            _ => None,
        }
    }
}

pub fn is_drivable(block: Block) -> bool {
    matches!(block, Block::Lever { .. })
        || block.button().is_some()
        || block.pressure_plate().is_some()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum InputError {
    #[error("is not an input")]
    NotAnInput,
    #[error("can't take {0:?}")]
    Unsupported(Input),
}
