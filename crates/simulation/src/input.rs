use crate::ticks::SimulationError;
use mchprs_redstone::input::InputError;

#[derive(Debug, thiserror::Error)]
pub enum DriveError {
    #[error("the driven component {0}")]
    Refused(#[from] InputError),
    #[error(transparent)]
    Simulation(#[from] SimulationError),
}
