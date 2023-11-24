use std::num::NonZeroU8;

use mchprs_blocks::blocks::ComparatorMode;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(super) struct NonMaxU8(NonZeroU8);

impl NonMaxU8 {
    pub(super) fn new(n: u8) -> Option<Self> {
        Some(Self(NonZeroU8::new(n.wrapping_add(1))?))
    }

    pub(super) fn get(self) -> u8 {
        self.0.get() - 1
    }
}

pub(super) fn calculate_comparator_output(
    mode: ComparatorMode,
    input_strength: u8,
    power_on_sides: u8,
) -> u8 {
    match mode {
        ComparatorMode::Compare => {
            if input_strength >= power_on_sides {
                input_strength
            } else {
                0
            }
        }
        ComparatorMode::Subtract => input_strength.saturating_sub(power_on_sides),
    }
}
