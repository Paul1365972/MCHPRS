//! The reconfiguration port of a 7-series PLL (PLLE2), by XAPP888.

use std::ops::RangeInclusive;

const MULTIPLIERS: RangeInclusive<u32> = 2..=64;
const INPUT_DIVIDERS: RangeInclusive<u32> = 1..=56;
const OUTPUT_DIVIDERS: RangeInclusive<u32> = 1..=128;

const LOCK_DETECTORS: [u64; 35] = [
    0x31_be8f_a401,
    0x42_3e8f_a401,
    0x5a_fe8f_a401,
    0x73_be8f_a401,
    0x8c_7e8f_a401,
    0x9c_fe8f_a401,
    0xb5_be8f_a401,
    0xce_7e8f_a401,
    0xe7_3e8f_a401,
    0xff_f84f_a401,
    0xff_f39f_a401,
    0xff_eeef_a401,
    0xff_ebcf_a401,
    0xff_e8af_a401,
    0xff_e71f_a401,
    0xff_e3ff_a401,
    0xff_e26f_a401,
    0xff_e0df_a401,
    0xff_df4f_a401,
    0xff_ddbf_a401,
    0xff_dc2f_a401,
    0xff_da9f_a401,
    0xff_d90f_a401,
    0xff_d90f_a401,
    0xff_d77f_a401,
    0xff_d5ef_a401,
    0xff_d5ef_a401,
    0xff_d45f_a401,
    0xff_d45f_a401,
    0xff_d2cf_a401,
    0xff_d2cf_a401,
    0xff_d2cf_a401,
    0xff_d13f_a401,
    0xff_d13f_a401,
    0xff_d13f_a401,
];
const LAST_LOCK_DETECTOR: u64 = 0xff_cfaf_a401;
const LOOP_FILTERS: [u16; 63] = [
    0x0dc, 0x17c, 0x1fc, 0x1ec, 0x35c, 0x3ac, 0x3b4, 0x3f4, 0x3dc, 0x3ec, 0x3f4, 0x3cc, 0x394,
    0x3d4, 0x3d4, 0x3d4, 0x3d4, 0x1d8, 0x1d8, 0x1d8, 0x1d8, 0x170, 0x170, 0x170, 0x304, 0x304,
    0x304, 0x304, 0x304, 0x304, 0x304, 0x304, 0x108, 0x108, 0x108, 0x0a0, 0x0a0, 0x0a0, 0x0d0,
    0x0a0, 0x0a0, 0x0a0, 0x0a0, 0x0a0, 0x0a0, 0x0a0, 0x0a0, 0x0a0, 0x0a0, 0x0a0, 0x0a0, 0x0a0,
    0x130, 0x130, 0x130, 0x130, 0x130, 0x130, 0x130, 0x090, 0x090, 0x090, 0x090,
];

#[derive(Debug, Clone)]
pub struct PllLimits {
    pub vco_hertz: RangeInclusive<u64>,
    pub phase_detector_hertz: RangeInclusive<u64>,
    pub global_buffer_maximum_hertz: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PllSetting {
    multiplier: u32,
    input_divider: u32,
    output_divider: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PllRegister {
    pub address: u8,
    pub kept_mask: u16,
    pub value: u16,
}

impl PllSetting {
    /// Of equal outputs it has the fastest oscillator and then the fastest phase detector, which
    /// jitter least.
    pub fn fastest(input_hertz: u32, limits: &PllLimits, maximum_hertz: f64) -> Option<PllSetting> {
        let settings = MULTIPLIERS.flat_map(|multiplier| {
            INPUT_DIVIDERS.flat_map(move |input_divider| {
                OUTPUT_DIVIDERS.map(move |output_divider| PllSetting {
                    multiplier,
                    input_divider,
                    output_divider,
                })
            })
        });
        settings
            .filter(|setting| setting.is_within(input_hertz, limits))
            .filter(|setting| setting.output_hertz(input_hertz) <= maximum_hertz)
            .max_by(|setting, other| {
                let output = |setting: &PllSetting| setting.output_hertz(input_hertz);
                let oscillator = |setting: &PllSetting| setting.oscillator_hertz(input_hertz);
                let detector = |setting: &PllSetting| setting.phase_detector_hertz(input_hertz);
                output(setting)
                    .total_cmp(&output(other))
                    .then(oscillator(setting).total_cmp(&oscillator(other)))
                    .then(detector(setting).total_cmp(&detector(other)))
            })
    }

    /// Equal for settings of the same ratio: every product is exact and the division rounds once.
    pub fn output_hertz(self, input_hertz: u32) -> f64 {
        f64::from(input_hertz) * f64::from(self.multiplier)
            / f64::from(self.input_divider * self.output_divider)
    }

    fn oscillator_hertz(self, input_hertz: u32) -> f64 {
        f64::from(input_hertz) * f64::from(self.multiplier) / f64::from(self.input_divider)
    }

    fn phase_detector_hertz(self, input_hertz: u32) -> f64 {
        f64::from(input_hertz) / f64::from(self.input_divider)
    }

    fn is_within(self, input_hertz: u32, limits: &PllLimits) -> bool {
        let input = u64::from(input_hertz);
        let input_divider = u64::from(self.input_divider);
        let multiplied = input * u64::from(self.multiplier);
        let within = |range: &RangeInclusive<u64>, hertz: u64| {
            range.start() * input_divider <= hertz && hertz <= range.end() * input_divider
        };
        within(&limits.phase_detector_hertz, input)
            && within(&limits.vco_hertz, multiplied)
            && multiplied
                <= limits.global_buffer_maximum_hertz
                    * input_divider
                    * u64::from(self.output_divider)
    }

    pub fn registers(self) -> [PllRegister; 10] {
        let index = (self.multiplier - MULTIPLIERS.start()) as usize;
        let lock = LOCK_DETECTORS.get(index).unwrap_or(&LAST_LOCK_DETECTOR);
        let filter = LOOP_FILTERS[index];
        let lock_bits = |low: u32, count: u32| (lock >> low) as u16 & ((1 << count) - 1);
        let filter_bit = |bit: u32, position: u32| (filter >> bit & 1) << position;
        let (output_times, output_flags) = counter(self.output_divider);
        let (feedback_times, feedback_flags) = counter(self.multiplier);
        let (input_times, input_flags) = counter(self.input_divider);
        [
            (0x08, 0x1000, output_times),
            (0x09, 0xfc00, output_flags << 6),
            (0x14, 0x1000, feedback_times),
            (0x15, 0xfc00, feedback_flags << 6),
            (0x16, 0xc000, input_flags << 12 | input_times),
            (0x18, 0xfc00, lock_bits(20, 10)),
            (0x19, 0x8000, lock_bits(30, 5) << 10 | lock_bits(0, 10)),
            (0x1a, 0x8000, lock_bits(35, 5) << 10 | lock_bits(10, 10)),
            (
                0x4e,
                0x66ff,
                filter_bit(9, 15) | filter_bit(8, 12) | filter_bit(7, 11) | filter_bit(6, 8),
            ),
            (
                0x4f,
                0x666f,
                filter_bit(5, 15)
                    | filter_bit(4, 12)
                    | filter_bit(3, 11)
                    | filter_bit(2, 8)
                    | filter_bit(1, 7)
                    | filter_bit(0, 4),
            ),
        ]
        .map(|(address, kept_mask, value)| PllRegister {
            address,
            kept_mask,
            value,
        })
    }
}

fn counter(divider: u32) -> (u16, u16) {
    let high = (divider / 2).max(1);
    let low = divider - divider / 2;
    let no_count = divider == 1;
    let edge = divider % 2 == 1 && !no_count;
    let flags = u16::from(edge) << 1 | u16::from(no_count);
    ((((high % 64) << 6) | (low % 64)) as u16, flags)
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn setting(multiplier: u32, input_divider: u32, output_divider: u32) -> PllSetting {
        PllSetting {
            multiplier,
            input_divider,
            output_divider,
        }
    }

    /// What the board read from the PLL of nextpnr's bitstreams, under the bits that a write keeps.
    #[test]
    fn the_registers_are_those_of_nextpnr() {
        const ADDRESSES: [u8; 10] = [0x08, 0x09, 0x14, 0x15, 0x16, 0x18, 0x19, 0x1a, 0x4e, 0x4f];
        let read = [
            (
                12,
                [
                    0x1186, 0x0000, 0x1186, 0x0000, 0x1041, 0x0339, 0x7c01, 0xffe9, 0x9908, 0x9100,
                ],
            ),
            (
                13,
                [
                    0x1187, 0x0080, 0x1187, 0x0080, 0x1041, 0x02ee, 0x7c01, 0xffe9, 0x9908, 0x0900,
                ],
            ),
            (
                16,
                [
                    0x1208, 0x0000, 0x1208, 0x0000, 0x1041, 0x0271, 0x7c01, 0xffe9, 0x9908, 0x1100,
                ],
            ),
        ];
        for (multiplier, read) in read {
            let registers = setting(multiplier, 1, multiplier).registers();
            assert_eq!(registers.map(|register| register.address), ADDRESSES);
            for (register, read) in registers.into_iter().zip(read) {
                assert_eq!(register.value & register.kept_mask, 0, "{register:x?}");
                assert_eq!(register.value, read & !register.kept_mask, "{register:x?}");
            }
        }
    }
}
