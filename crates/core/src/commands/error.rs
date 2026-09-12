use thiserror::Error;

pub(super) type CommandResult<T> = Result<T, CommandError>;

#[derive(Debug, Error)]
pub enum InternalError {
    #[error("player index {index} is invalid (state management bug)")]
    InvalidPlayerIndex { index: usize },
}

#[derive(Debug, Error)]
pub(super) enum CommandError {
    #[error("{0}")]
    Runtime(String),
    #[error(transparent)]
    Internal(#[from] InternalError),
}

impl CommandError {
    pub(super) fn runtime(message: impl Into<String>) -> Self {
        Self::Runtime(message.into())
    }

    pub(super) fn coordinate_out_of_range() -> Self {
        Self::runtime("Coordinates are outside the supported range")
    }

    pub(super) fn destination_outside_plot() -> Self {
        Self::runtime("Destination is outside plot bounds")
    }

    pub(super) fn empty_clipboard() -> Self {
        Self::runtime("Your clipboard is empty. Use //copy first.")
    }
}
