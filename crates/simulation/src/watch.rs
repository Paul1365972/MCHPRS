use std::mem;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Condition {
    Becomes(u64),
    Leaves(u64),
    Changes,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bit<A> {
    At(A),
    Fixed(bool),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct BreakpointId(pub u32);

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Breakpoint<A> {
    pub id: BreakpointId,
    pub bits: Box<[Bit<A>]>,
    pub condition: Condition,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Hit {
    pub breakpoint: BreakpointId,
    pub tick: u64,
    pub value: u64,
}

/// Checked at the end of every tick, and a run stops after the tick in which a breakpoint fires. A
/// new watch keeps the state of each breakpoint the previous one had.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Watch<A> {
    pub breakpoints: Vec<Breakpoint<A>>,
}

#[derive(Debug, Clone)]
pub struct Breakpoints<A> {
    watched: Vec<Watched<A>>,
}

impl<A: Copy + PartialEq> Breakpoints<A> {
    pub fn new(
        breakpoints: Vec<Breakpoint<A>>,
        previous: Option<&Breakpoints<A>>,
        mut powered: impl FnMut(A) -> bool,
    ) -> Breakpoints<A> {
        let watched = breakpoints.into_iter().map(|breakpoint| {
            let kept = previous.and_then(|previous| previous.trigger(&breakpoint));
            let trigger = kept.unwrap_or_else(|| {
                Trigger::new(breakpoint.condition, value(&breakpoint.bits, &mut powered))
            });
            Watched {
                id: breakpoint.id,
                bits: breakpoint.bits,
                trigger,
            }
        });
        Breakpoints {
            watched: watched.collect(),
        }
    }

    pub fn fire(&mut self, index: usize, tick: u64, powered: impl FnMut(A) -> bool) -> Option<Hit> {
        let watched = &mut self.watched[index];
        let value = value(&watched.bits, powered);
        watched.trigger.fires(value).then_some(Hit {
            breakpoint: watched.id,
            tick,
            value,
        })
    }

    /// Fires every breakpoint even after a hit, since each one records the value it reads.
    pub fn fire_all(&mut self, tick: u64, mut powered: impl FnMut(A) -> bool) -> Option<Hit> {
        let mut hit = None;
        for index in 0..self.watched.len() {
            hit = hit.or(self.fire(index, tick, &mut powered));
        }
        hit
    }

    fn trigger(&self, breakpoint: &Breakpoint<A>) -> Option<Trigger> {
        let mut watched = self.watched.iter();
        let watched = watched.find(|watched| watched.id == breakpoint.id)?;
        let is_same =
            watched.bits == breakpoint.bits && watched.trigger.condition == breakpoint.condition;
        is_same.then_some(watched.trigger)
    }
}

impl<A> Default for Breakpoints<A> {
    fn default() -> Breakpoints<A> {
        Breakpoints {
            watched: Vec::new(),
        }
    }
}

#[derive(Debug, Clone)]
struct Watched<A> {
    id: BreakpointId,
    bits: Box<[Bit<A>]>,
    trigger: Trigger,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Trigger {
    condition: Condition,
    seen: u64,
}

impl Trigger {
    fn new(condition: Condition, value: u64) -> Trigger {
        Trigger {
            condition,
            seen: value,
        }
    }

    fn fires(&mut self, value: u64) -> bool {
        let seen = mem::replace(&mut self.seen, value);
        match self.condition {
            Condition::Becomes(expected) => value == expected && seen != expected,
            Condition::Leaves(left) => value != left && seen == left,
            Condition::Changes => value != seen,
        }
    }
}

fn value<A: Copy>(bits: &[Bit<A>], mut powered: impl FnMut(A) -> bool) -> u64 {
    bits.iter().enumerate().fold(0, |value, (index, &bit)| {
        let on = match bit {
            Bit::At(at) => powered(at),
            Bit::Fixed(on) => on,
        };
        value | u64::from(on) << index
    })
}
