use crate::watch::Hit;

pub type SimulationError = anyhow::Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ran {
    pub ticks: u64,
    pub stop: Option<Stop>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stop {
    Settled,
    Breakpoint(Hit),
}

pub fn run_until_hit<T: ?Sized>(
    target: &mut T,
    ticks: u64,
    mut tick: impl FnMut(&mut T) -> Option<Hit>,
) -> Ran {
    for ran in 1..=ticks {
        if let Some(hit) = tick(target) {
            return Ran {
                ticks: ran,
                stop: Some(Stop::Breakpoint(hit)),
            };
        }
    }
    Ran { ticks, stop: None }
}

pub fn settle_until_hit<T: ?Sized>(
    target: &mut T,
    limit: u64,
    is_idle: impl Fn(&T) -> bool,
    mut tick: impl FnMut(&mut T) -> Option<Hit>,
) -> Ran {
    let mut ticks = 0;
    while !is_idle(target) && ticks < limit {
        ticks += 1;
        if let Some(hit) = tick(target) {
            return Ran {
                ticks,
                stop: Some(Stop::Breakpoint(hit)),
            };
        }
    }
    Ran {
        ticks,
        stop: is_idle(target).then_some(Stop::Settled),
    }
}
