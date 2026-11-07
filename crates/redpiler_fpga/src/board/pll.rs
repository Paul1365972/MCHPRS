use super::BOARD_CLOCK_HERTZ;
use std::cmp::Ordering;
use std::collections::BTreeSet;
use std::ops::RangeInclusive;

/// The feedback multipliers of a 7 series PLLE2
const MULTIPLIERS: RangeInclusive<u32> = 2..=64;
/// The dividers of a 7 series PLLE2's input
const INPUT_DIVIDERS: RangeInclusive<u32> = 1..=56;
/// The dividers of a 7 series PLLE2's outputs
const OUTPUT_DIVIDERS: RangeInclusive<u32> = 1..=128;
/// The oscillator frequencies an XC7A35T-1's PLL allows
const OSCILLATOR_HERTZ: RangeInclusive<u64> = 800_000_000..=1_600_000_000;
/// The phase detector frequencies an XC7A35T-1's PLL allows
const PHASE_DETECTOR_HERTZ: RangeInclusive<u64> = 19_000_000..=450_000_000;
/// The fastest clock an XC7A35T-1's global clock buffers carry
const GLOBAL_CLOCK_MAXIMUM_HERTZ: u64 = 464_000_000;
/// The lock detector setting of each multiplier from 2, as Vivado programs it (harvested from
/// Vivado bitstreams for openXC7's nextpnr)
const LOCK_DETECTORS: [u64; 63] = [
    0x31BE8FA401,
    0x423E8FA401,
    0x5AFE8FA401,
    0x73BE8FA401,
    0x8C7E8FA401,
    0x9CFE8FA401,
    0xB5BE8FA401,
    0xCE7E8FA401,
    0xE73E8FA401,
    0xFFF84FA401,
    0xFFF39FA401,
    0xFFEEEFA401,
    0xFFEBCFA401,
    0xFFE8AFA401,
    0xFFE71FA401,
    0xFFE3FFA401,
    0xFFE26FA401,
    0xFFE0DFA401,
    0xFFDF4FA401,
    0xFFDDBFA401,
    0xFFDC2FA401,
    0xFFDA9FA401,
    0xFFD90FA401,
    0xFFD90FA401,
    0xFFD77FA401,
    0xFFD5EFA401,
    0xFFD5EFA401,
    0xFFD45FA401,
    0xFFD45FA401,
    0xFFD2CFA401,
    0xFFD2CFA401,
    0xFFD2CFA401,
    0xFFD13FA401,
    0xFFD13FA401,
    0xFFD13FA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
    0xFFCFAFA401,
];
/// The loop filter of each multiplier from 2 at the default bandwidth, as Vivado programs it
/// (harvested from Vivado bitstreams for openXC7's nextpnr)
const LOOP_FILTERS: [u16; 63] = [
    0x0DC, 0x17C, 0x1FC, 0x1EC, 0x35C, 0x3AC, 0x3B4, 0x3F4, 0x3DC, 0x3EC, 0x3F4, 0x3CC, 0x394,
    0x3D4, 0x3D4, 0x3D4, 0x3D4, 0x1D8, 0x1D8, 0x1D8, 0x1D8, 0x170, 0x170, 0x170, 0x304, 0x304,
    0x304, 0x304, 0x304, 0x304, 0x304, 0x304, 0x108, 0x108, 0x108, 0x0A0, 0x0A0, 0x0A0, 0x0D0,
    0x0A0, 0x0A0, 0x0A0, 0x0A0, 0x0A0, 0x0A0, 0x0A0, 0x0A0, 0x0A0, 0x0A0, 0x0A0, 0x0A0, 0x0A0,
    0x130, 0x130, 0x130, 0x130, 0x130, 0x130, 0x130, 0x090, 0x090, 0x090, 0x090,
];

#[derive(Debug, thiserror::Error)]
#[error("the routed design holds {0} PLLs instead of one")]
pub(crate) struct PllCountError(usize);

/// How the PLL derives the circuit clock from the board clock: its phase detector compares the
/// board clock divided by `input_divider` with its oscillator divided by `multiplier`, and
/// `output_divider` divides the oscillator into the circuit clock
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct PllSetting {
    multiplier: u32,
    input_divider: u32,
    output_divider: u32,
}

impl PllSetting {
    /// The setting with the fastest circuit clock up to `maximum_hertz`
    pub(super) fn fastest(maximum_hertz: f64) -> Option<PllSetting> {
        MULTIPLIERS
            .flat_map(|multiplier| {
                INPUT_DIVIDERS.flat_map(move |input_divider| {
                    OUTPUT_DIVIDERS.map(move |output_divider| PllSetting {
                        multiplier,
                        input_divider,
                        output_divider,
                    })
                })
            })
            .filter(|setting| setting.is_valid() && setting.output_hertz() <= maximum_hertz)
            .max_by(|setting, other| setting.compare_preference(*other))
    }

    pub(super) fn output_hertz(self) -> f64 {
        f64::from(BOARD_CLOCK_HERTZ) * f64::from(self.multiplier)
            / f64::from(self.input_divider * self.output_divider)
    }

    fn is_valid(self) -> bool {
        let board_clock_hertz = u64::from(BOARD_CLOCK_HERTZ);
        let input_divider = u64::from(self.input_divider);
        let multiplied_hertz = board_clock_hertz * u64::from(self.multiplier);
        let output_divider = u64::from(self.output_divider);
        PHASE_DETECTOR_HERTZ.start() * input_divider <= board_clock_hertz
            && board_clock_hertz <= PHASE_DETECTOR_HERTZ.end() * input_divider
            && OSCILLATOR_HERTZ.start() * input_divider <= multiplied_hertz
            && multiplied_hertz <= OSCILLATOR_HERTZ.end() * input_divider
            && multiplied_hertz <= GLOBAL_CLOCK_MAXIMUM_HERTZ * input_divider * output_divider
    }

    /// Orders by the circuit clock, then by the oscillator and then by the phase detector, as the
    /// faster ones jitter less
    fn compare_preference(self, other: PllSetting) -> Ordering {
        let output = |setting: PllSetting, by: PllSetting| {
            u64::from(setting.multiplier) * u64::from(by.input_divider * by.output_divider)
        };
        let oscillator = |setting: PllSetting, by: PllSetting| {
            u64::from(setting.multiplier) * u64::from(by.input_divider)
        };
        output(self, other)
            .cmp(&output(other, self))
            .then(oscillator(self, other).cmp(&oscillator(other, self)))
            .then(other.input_divider.cmp(&self.input_divider))
    }

    /// The routed design's assembly with this setting in place of the PLL's own
    pub(super) fn apply(self, assembly: &str) -> Result<String, PllCountError> {
        let tiles: BTreeSet<&str> = assembly
            .lines()
            .filter_map(|line| line.strip_suffix(".PLLE2_ADV.IN_USE"))
            .collect();
        let [tile] = tiles.iter().copied().collect::<Vec<_>>()[..] else {
            return Err(PllCountError(tiles.len()));
        };
        let pll = format!("{tile}.PLLE2_ADV.");
        let counters = [
            (Counter::Input, self.input_divider),
            (Counter::Feedback, self.multiplier),
            (Counter::Output, self.output_divider),
        ];
        let replaced: Vec<String> = counters
            .iter()
            .flat_map(|(counter, _)| counter.fields())
            .chain(["LKTABLE".to_owned(), "TABLE".to_owned()])
            .map(|field| format!("{pll}{field}["))
            .collect();
        let mut lines: Vec<String> = assembly
            .lines()
            .filter(|line| !replaced.iter().any(|field| line.starts_with(field)))
            .map(str::to_owned)
            .collect();
        for (counter, divider) in counters {
            lines.extend(counter.lines(&pll, divider));
        }
        let multiplier = (self.multiplier - MULTIPLIERS.start()) as usize;
        let lock_detector = LOCK_DETECTORS[multiplier];
        let loop_filter = LOOP_FILTERS[multiplier];
        lines.push(format!("{pll}LKTABLE[39:0] = 40'b{lock_detector:040b}"));
        lines.push(format!("{pll}TABLE[9:0] = 10'b{loop_filter:010b}"));
        lines.push(String::new());
        Ok(lines.join("\n"))
    }
}

/// A counter of the PLL that divides a clock
#[derive(Clone, Copy)]
enum Counter {
    Input,
    Feedback,
    Output,
}

impl Counter {
    /// The assembly fields of its high and low times, edge bit and bypass
    fn fields(self) -> [String; 4] {
        let (times, flags) = match self {
            Counter::Input => ("DIVCLK_DIVCLK", "DIVCLK_DIVCLK"),
            Counter::Feedback => ("CLKFBOUT_CLKOUT1", "CLKFBOUT_CLKOUT2"),
            Counter::Output => ("CLKOUT0_CLKOUT1", "CLKOUT0_CLKOUT2"),
        };
        [
            format!("{times}_HIGH_TIME"),
            format!("{times}_LOW_TIME"),
            format!("{flags}_EDGE"),
            format!("{flags}_NO_COUNT"),
        ]
    }

    /// The lines that make it divide by `divider` at half duty. A divider of one bypasses the
    /// counter; any other splits each period into a high and a low time, and an odd one sets the
    /// edge bit for its extra half period. The six-bit times drop the 64 of a divider of 127 or 128.
    fn lines(self, pll: &str, divider: u32) -> Vec<String> {
        let [high_time, low_time, edge, no_count] = self.fields();
        let (high, low) = match divider {
            1 => (1, 1),
            _ => (divider / 2, divider - divider / 2),
        };
        let mut lines = vec![
            format!("{pll}{high_time}[5:0] = 6'b{:06b}", high % 64),
            format!("{pll}{low_time}[5:0] = 6'b{:06b}", low % 64),
        ];
        if divider == 1 {
            lines.push(format!("{pll}{no_count}[0]"));
        } else if divider % 2 == 1 {
            lines.push(format!("{pll}{edge}[0]"));
        }
        lines
    }
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

    /// nextpnr's PLL lines for each setting of a blinking test design
    const NEXTPNR_ASSEMBLIES: [(PllSetting, &str); 6] = [
        (setting(16, 1, 16), include_str!("pll/16-1-16.fasm")),
        (setting(13, 1, 12), include_str!("pll/13-1-12.fasm")),
        (setting(34, 3, 15), include_str!("pll/34-3-15.fasm")),
        (setting(18, 2, 9), include_str!("pll/18-2-9.fasm")),
        (setting(64, 5, 127), include_str!("pll/64-5-127.fasm")),
        (setting(8, 1, 128), include_str!("pll/8-1-128.fasm")),
    ];

    #[test]
    fn settings_follow_the_assembly_of_nextpnr() {
        let other_cell = "CLBLM_R_X3Y5.SLICEM_X0.AFF.ZINI\n";
        let lines =
            |assembly: &str| -> BTreeSet<String> { assembly.lines().map(str::to_owned).collect() };
        for (_, routed) in NEXTPNR_ASSEMBLIES {
            for (setting, expected) in NEXTPNR_ASSEMBLIES {
                let applied = setting.apply(&format!("{routed}{other_cell}")).unwrap();
                let expected = format!("{expected}{other_cell}");
                assert_eq!(lines(&applied), lines(&expected), "{setting:?}");
            }
        }
    }

    #[test]
    fn assemblies_need_exactly_one_pll() {
        let (_, routed) = NEXTPNR_ASSEMBLIES[0];
        let twice = format!("{routed}{}", routed.replace("X106Y44", "X106Y96"));
        let setting = setting(16, 1, 16);
        assert!(matches!(setting.apply(&twice), Err(PllCountError(2))));
        assert!(matches!(setting.apply(""), Err(PllCountError(0))));
    }

    #[test]
    fn the_circuit_clock_takes_the_fastest_setting_it_meets() {
        assert_eq!(PllSetting::fastest(108.6e6), Some(setting(13, 1, 12)));
        assert_eq!(PllSetting::fastest(98.2e6), Some(setting(54, 5, 11)));
        assert_eq!(PllSetting::fastest(75.7e6), Some(setting(34, 3, 15)));
        assert_eq!(PllSetting::fastest(1e9), Some(setting(37, 4, 2)));
        assert_eq!(PllSetting::fastest(6.25e6), Some(setting(8, 1, 128)));
        assert_eq!(PllSetting::fastest(6.2e6), None);
    }

    #[test]
    fn equal_clocks_prefer_the_faster_oscillator_and_phase_detector() {
        assert_eq!(PllSetting::fastest(100e6), Some(setting(16, 1, 16)));
        assert_eq!(PllSetting::fastest(50e6), Some(setting(16, 1, 32)));
    }
}
