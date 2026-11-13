use std::ops::RangeInclusive;

pub struct Rng {
    state: u64,
}

impl Rng {
    pub fn new(seed: u64) -> Rng {
        // Xorshift never leaves a state of zero
        Rng {
            state: splitmix64(seed).max(1),
        }
    }

    fn next(&mut self) -> u64 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 7;
        self.state ^= self.state << 17;
        self.state
    }

    pub fn below(&mut self, bound: u64) -> u64 {
        self.next() % bound
    }

    pub fn within<T: Into<u64> + TryFrom<u64>>(&mut self, range: RangeInclusive<T>) -> T {
        let (start, end) = range.into_inner();
        let (start, end): (u64, u64) = (start.into(), end.into());
        let value = start + self.below(end - start + 1);
        T::try_from(value).unwrap_or_else(|_| unreachable!("a value within a range fits its type"))
    }

    pub fn chance(&mut self, percent: u64) -> bool {
        self.below(100) < percent
    }

    pub fn pick<T: Clone>(&mut self, items: &[T]) -> T {
        items[self.below(items.len() as u64) as usize].clone()
    }

    pub fn shuffle<T>(&mut self, items: &mut [T]) {
        for index in (1..items.len()).rev() {
            items.swap(index, self.below(index as u64 + 1) as usize);
        }
    }
}

fn splitmix64(value: u64) -> u64 {
    let value = value.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let value = (value ^ (value >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    let value = (value ^ (value >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    value ^ (value >> 31)
}
