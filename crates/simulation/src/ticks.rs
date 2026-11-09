pub type SimulationError = anyhow::Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ran {
    pub ticks: u64,
    pub stop: Option<Stop>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stop {
    Settled,
}
