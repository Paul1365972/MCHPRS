use serde::{Deserialize, Deserializer, Serialize};
use std::fmt;
use std::str::FromStr;

#[repr(u8)]
#[derive(
    Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
#[serde(try_from = "u8", into = "u8")]
pub enum SignalStrength {
    #[default]
    Zero = 0,
    One,
    Two,
    Three,
    Four,
    Five,
    Six,
    Seven,
    Eight,
    Nine,
    Ten,
    Eleven,
    Twelve,
    Thirteen,
    Fourteen,
    Fifteen,
}

impl SignalStrength {
    #[inline]
    pub const fn get(self) -> u8 {
        self as u8
    }

    #[inline]
    pub const fn is_zero(self) -> bool {
        matches!(self, Self::Zero)
    }

    #[inline]
    pub fn saturating_sub(self, amount: u8) -> Self {
        Self::try_from(self.get().saturating_sub(amount)).unwrap()
    }

    #[inline]
    pub fn saturating_from(value: i32) -> Self {
        Self::try_from(value.clamp(0, 15) as u32).unwrap()
    }

    pub fn deserialize_saturating<'de, D: Deserializer<'de>>(
        deserializer: D,
    ) -> Result<Self, D::Error> {
        u8::deserialize(deserializer).map(|value| Self::saturating_from(value.into()))
    }
}

impl From<bool> for SignalStrength {
    #[inline]
    fn from(powered: bool) -> Self {
        if powered {
            Self::Fifteen
        } else {
            Self::Zero
        }
    }
}

impl From<SignalStrength> for u8 {
    #[inline]
    fn from(power: SignalStrength) -> Self {
        power.get()
    }
}

impl TryFrom<u8> for SignalStrength {
    type Error = InvalidSignalStrength;

    #[inline]
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Self::try_from(u32::from(value))
    }
}

impl TryFrom<u32> for SignalStrength {
    type Error = InvalidSignalStrength;

    #[inline]
    fn try_from(value: u32) -> Result<Self, Self::Error> {
        Ok(match value {
            0 => Self::Zero,
            1 => Self::One,
            2 => Self::Two,
            3 => Self::Three,
            4 => Self::Four,
            5 => Self::Five,
            6 => Self::Six,
            7 => Self::Seven,
            8 => Self::Eight,
            9 => Self::Nine,
            10 => Self::Ten,
            11 => Self::Eleven,
            12 => Self::Twelve,
            13 => Self::Thirteen,
            14 => Self::Fourteen,
            15 => Self::Fifteen,
            _ => return Err(InvalidSignalStrength(value)),
        })
    }
}

impl TryFrom<UncappedSignalStrength> for SignalStrength {
    type Error = InvalidSignalStrength;

    #[inline]
    fn try_from(value: UncappedSignalStrength) -> Result<Self, Self::Error> {
        Self::try_from(u32::from(value.get()))
    }
}

impl FromStr for SignalStrength {
    type Err = ();

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        let value: u8 = value.parse().map_err(|_| ())?;
        Self::try_from(value).map_err(|_| ())
    }
}

impl fmt::Display for SignalStrength {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.get().fmt(formatter)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct UncappedSignalStrength(u16);

impl UncappedSignalStrength {
    pub const ZERO: Self = Self(0);

    #[inline]
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    #[inline]
    pub const fn get(self) -> u16 {
        self.0
    }

    #[inline]
    pub const fn is_zero(self) -> bool {
        self.0 == 0
    }

    #[inline]
    pub fn capped(self) -> SignalStrength {
        SignalStrength::try_from(u32::from(self.0.min(15))).unwrap()
    }

    #[inline]
    pub fn saturating_sub(self, other: Self) -> Self {
        Self(self.0.saturating_sub(other.0))
    }
}

impl From<SignalStrength> for UncappedSignalStrength {
    #[inline]
    fn from(power: SignalStrength) -> Self {
        Self(power.get().into())
    }
}

#[derive(Debug)]
pub struct InvalidSignalStrength(u32);

impl fmt::Display for InvalidSignalStrength {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "signal strength {} is outside 0..=15", self.0)
    }
}

impl std::error::Error for InvalidSignalStrength {}
