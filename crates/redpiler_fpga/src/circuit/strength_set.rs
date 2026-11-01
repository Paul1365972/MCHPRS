use mchprs_redpiler::compile_graph::SignalStrength;
use std::ops::{BitOr, BitOrAssign};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct StrengthSet(u16);

impl StrengthSet {
    pub(crate) const OFF: Self = Self::only(SignalStrength::Zero);
    pub(crate) const ON: Self = Self::only(SignalStrength::Fifteen);

    pub(crate) const fn only(strength: SignalStrength) -> Self {
        Self(1 << strength.get())
    }

    pub(crate) fn contains(self, strength: SignalStrength) -> bool {
        self.0 & 1 << strength.get() != 0
    }

    pub(crate) fn insert(&mut self, strength: SignalStrength) {
        self.0 |= 1 << strength.get();
    }

    pub(crate) fn remove(&mut self, strength: SignalStrength) {
        self.0 &= !(1 << strength.get());
    }

    pub(crate) fn len(self) -> u32 {
        self.0.count_ones()
    }

    pub(crate) fn minimum(self) -> Option<SignalStrength> {
        (self.0 != 0).then(|| strength(self.0.trailing_zeros()))
    }

    pub(crate) fn maximum(self) -> Option<SignalStrength> {
        (self.0 != 0).then(|| strength(u16::BITS - 1 - self.0.leading_zeros()))
    }

    pub(crate) fn single(self) -> Option<SignalStrength> {
        (self.len() == 1).then(|| strength(self.0.trailing_zeros()))
    }

    pub(crate) fn iter(self) -> impl Iterator<Item = SignalStrength> {
        (0..u16::BITS)
            .filter(move |bit| self.0 & 1 << bit != 0)
            .map(strength)
    }

    pub(crate) fn attenuated(self, attenuation: u8) -> Self {
        self.iter()
            .map(|strength| strength.saturating_sub(attenuation))
            .collect()
    }

    pub(crate) fn at_least(self, floor: SignalStrength) -> Self {
        Self(self.0 & !((1 << floor.get()) - 1))
    }
}

fn strength(bit: u32) -> SignalStrength {
    SignalStrength::try_from(bit).expect("a strength set has 16 bits")
}

impl BitOr for StrengthSet {
    type Output = Self;

    fn bitor(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

impl BitOrAssign for StrengthSet {
    fn bitor_assign(&mut self, other: Self) {
        self.0 |= other.0;
    }
}

impl FromIterator<SignalStrength> for StrengthSet {
    fn from_iter<I: IntoIterator<Item = SignalStrength>>(strengths: I) -> Self {
        strengths
            .into_iter()
            .fold(Self::default(), |set, strength| set | Self::only(strength))
    }
}
