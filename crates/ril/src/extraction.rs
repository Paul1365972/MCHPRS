use mchprs_redpiler::extract::ExtractOptions;
use mchprs_redpiler::ports::WireOutputs;
use mchprs_syntax::flag::{Flag, FlagValues};

pub const IO_ONLY: Flag = Flag::switch(
    "io-only",
    Some('i'),
    "Only update the blocks of inputs, outputs and probed components",
);

pub const FLAGS: [Flag; 4] = [
    IO_ONLY,
    WireOutputs::DOTS,
    WireOutputs::ILLEGAL_STATES,
    WireOutputs::CROSSES,
];

pub fn options(flags: &FlagValues) -> ExtractOptions {
    ExtractOptions {
        wire_outputs: WireOutputs::from_flags(flags),
        ports_only: flags.is_on(&IO_ONLY),
        ..ExtractOptions::default()
    }
}
