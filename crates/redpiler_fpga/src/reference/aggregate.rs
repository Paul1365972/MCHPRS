#[derive(Clone, Default)]
pub(super) struct Aggregate {
    counts: [usize; 16],
    occupied: u16,
}

impl Aggregate {
    pub(super) fn insert(&mut self, strength: u8) {
        self.counts[usize::from(strength)] += 1;
        self.occupied |= 1 << strength;
    }

    pub(super) fn replace(&mut self, previous: u8, strength: u8) {
        self.counts[usize::from(previous)] -= 1;
        if self.counts[usize::from(previous)] == 0 {
            self.occupied &= !(1 << previous);
        }
        self.insert(strength);
    }

    pub(super) fn maximum(&self) -> u8 {
        (15 - (self.occupied | 1).leading_zeros()) as u8
    }
}
