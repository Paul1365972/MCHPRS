//! The dividers of a 7-series PLL (PLLE2), written through its reconfiguration port by XAPP888.
//! The multiplier stays as the bitstream has it, so nothing else of the PLL depends on a setting.

use std::ops::RangeInclusive;

// The ranges of a PLLE2's `DIVCLK_DIVIDE` and `CLKOUT0_DIVIDE` (UG953).
const INPUT_DIVIDERS: RangeInclusive<u32> = 1..=56;
const OUTPUT_DIVIDERS: RangeInclusive<u32> = 1..=128;

#[derive(Debug, Clone)]
pub struct Pll {
    pub multiplier: u32,
    pub vco_hertz: RangeInclusive<u64>,
    pub phase_detector_hertz: RangeInclusive<u64>,
    pub global_buffer_maximum_hertz: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PllSetting {
    pub input_divider: u32,
    pub output_divider: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PllRegister {
    pub address: u8,
    pub kept_mask: u16,
    pub value: u16,
}

impl Pll {
    /// Of equal outputs it has the fastest VCO, which jitters least.
    pub fn fastest_setting(&self, input_hertz: u32, maximum_hertz: f64) -> Option<PllSetting> {
        let settings = INPUT_DIVIDERS.flat_map(|input_divider| {
            OUTPUT_DIVIDERS.map(move |output_divider| PllSetting {
                input_divider,
                output_divider,
            })
        });
        settings
            .filter(|&setting| self.allows(setting, input_hertz))
            .filter(|&setting| self.output_hertz(setting, input_hertz) <= maximum_hertz)
            .max_by(|&setting, &other| {
                let output = |setting| self.output_hertz(setting, input_hertz);
                output(setting)
                    .total_cmp(&output(other))
                    .then(other.input_divider.cmp(&setting.input_divider))
            })
    }

    /// Equal for settings of the same ratio: every product is exact and the division rounds once.
    pub fn output_hertz(&self, setting: PllSetting, input_hertz: u32) -> f64 {
        f64::from(input_hertz) * f64::from(self.multiplier)
            / f64::from(setting.input_divider * setting.output_divider)
    }

    fn allows(&self, setting: PllSetting, input_hertz: u32) -> bool {
        let input = u64::from(input_hertz);
        let input_divider = u64::from(setting.input_divider);
        let multiplied = input * u64::from(self.multiplier);
        let within = |range: &RangeInclusive<u64>, hertz: u64| {
            range.start() * input_divider <= hertz && hertz <= range.end() * input_divider
        };
        within(&self.phase_detector_hertz, input)
            && within(&self.vco_hertz, multiplied)
            && multiplied
                <= self.global_buffer_maximum_hertz
                    * input_divider
                    * u64::from(setting.output_divider)
    }
}

impl PllSetting {
    pub fn registers(self) -> [PllRegister; 3] {
        let (output_times, output_flags) = counter(self.output_divider);
        let (input_times, input_flags) = counter(self.input_divider);
        // XAPP888 v1.8, Table 15: the two registers of CLKOUT0 and the register of DIVCLK. Each
        // mask keeps the bits that the note reserves on a PLL (Tables 1, 2 and 8).
        [
            (0x08, 0x1000, output_times),
            (0x09, 0xfc00, output_flags << 6),
            (0x16, 0xc000, input_flags << 12 | input_times),
        ]
        .map(|(address, kept_mask, value)| PllRegister {
            address,
            kept_mask,
            value,
        })
    }
}

/// A divider counts the clocks its output is high and low for, 64 as 0. The flags are EDGE, which
/// ends the high time half a clock late for an odd divider, and NO COUNT for the divider 1, which
/// no count expresses (XAPP888, Tables 1, 2 and 8).
fn counter(divider: u32) -> (u16, u16) {
    let high = (divider / 2).max(1);
    let low = divider - divider / 2;
    let no_count = divider == 1;
    let edge = divider % 2 == 1 && !no_count;
    let flags = u16::from(edge) << 1 | u16::from(no_count);
    ((((high % 64) << 6) | (low % 64)) as u16, flags)
}
