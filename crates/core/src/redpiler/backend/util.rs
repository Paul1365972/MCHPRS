use std::num::NonZeroU8;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct NonMaxU8(NonZeroU8);

impl NonMaxU8 {
    pub fn new(n: u8) -> Option<Self> {
        Some(Self(NonZeroU8::new(n.wrapping_add(1))?))
    }

    pub fn get(self) -> u8 {
        self.0.get() - 1
    }
}
