//! A request is `FRAME_END`, an opcode, an argument of `ARGUMENT_BYTES` and a CRC-16, then
//! `FRAME_END`. An answer is a status byte, a payload and a CRC-16, then `FRAME_END`. Everything
//! between two `FRAME_END`s is byte-stuffed. `rtl/pkg.sv` holds the same constants for the board.

use anyhow::ensure;
use mchprs_redstone::input::Input;

pub const FRAME_END: u8 = 0xc0;
const ESCAPE: u8 = 0xdb;
const ESCAPED_END: u8 = 0xdc;
const ESCAPED_ESCAPE: u8 = 0xdd;
const STATUS_OK: u8 = 0;
const CHECKSUM_BYTES: usize = size_of::<u16>();
const ARGUMENT_BYTES: usize = 6;
const REQUEST_BYTES: usize = 1 + ARGUMENT_BYTES;
const COUNT_BITS: u32 = 40;
/// The most ticks a counted run and the most clocks per tick a periodic run can have.
pub const MAX_COUNT: u64 = (1 << COUNT_BITS) - 1;
pub const DRIVE_NODE_BITS: u32 = 24;
const INPUT_KIND_BITS: u32 = 2;
const INPUT_VALUE_BITS: u32 = 4;
pub const DRIVE_INPUT_BITS: u32 = INPUT_KIND_BITS + INPUT_VALUE_BITS;

macro_rules! codes {
    ($(#[$meta:meta])* $name:ident { $($(#[$variant_meta:meta])* $variant:ident = $code:literal,)* }) => {
        $(#[$meta])*
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub enum $name {
            $($(#[$variant_meta])* $variant = $code,)*
        }

        impl $name {
            pub const ALL: &[$name] = &[$($name::$variant,)*];
        }

        impl From<$name> for u64 {
            fn from(code: $name) -> u64 {
                code as u64
            }
        }
    };
}

codes!(Opcode {
    Hello = 0,
    Run = 1,
    Drive = 2,
    Read = 3,
});

codes!(ModeKind {
    Paused = 0,
    Unlimited = 1,
    Counted = 2,
    Periodic = 3,
});

codes!(InputKind {
    Set = 0,
    Toggle = 1,
    Press = 2,
    Touch = 3,
});

codes!(
    /// A read clears the events it reports, such as played note blocks.
    Contents {
        Shown = 0,
        All = 1,
    }
);

codes!(
    #[derive(thiserror::Error)]
    Fault {
        #[error("the request's checksum is wrong")]
        BadChecksum = 1,
        #[error("the request has the wrong length")]
        BadLength = 2,
        #[error("the request has a broken escape sequence")]
        BadEscape = 3,
        #[error("the opcode is unknown")]
        UnknownOpcode = 4,
    }
);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Request {
    Hello,
    Run(Mode),
    Drive { node: u32, input: Input },
    Read(Contents),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Paused,
    Unlimited,
    Counted { ticks: u64 },
    Periodic { clocks_per_tick: u64 },
}

/// The tick count and the state, from one tick boundary.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Readout {
    pub ticks: u64,
    pub state: Vec<u8>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Hello {
    pub build_nonce: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum AnswerError {
    #[error("the answer has a broken escape sequence")]
    Escape,
    #[error("the answer ends after {0} bytes")]
    Truncated(usize),
    #[error("the answer's checksum is wrong")]
    Checksum,
    #[error("the board refused the request: {0}")]
    Fault(#[from] Fault),
    #[error("the answer has the unknown status {0}")]
    UnknownStatus(u8),
}

impl Request {
    /// Packs the fields of `pkg::run_t` and `pkg::drive_t` as the board unpacks them, the first
    /// field highest.
    pub fn encode(self) -> [u8; REQUEST_BYTES] {
        let fitting = |value: u64, bits: u32| {
            assert!(value < 1 << bits, "{self:?} does not fit a request");
            value
        };
        let (opcode, argument) = match self {
            Request::Hello => (Opcode::Hello, 0),
            Request::Run(mode) => {
                let (kind, count) = match mode {
                    Mode::Paused => (ModeKind::Paused, 0),
                    Mode::Unlimited => (ModeKind::Unlimited, 0),
                    Mode::Counted { ticks } => (ModeKind::Counted, ticks),
                    Mode::Periodic { clocks_per_tick } => (ModeKind::Periodic, clocks_per_tick),
                };
                let count = fitting(count, COUNT_BITS);
                (Opcode::Run, u64::from(kind) << COUNT_BITS | count)
            }
            Request::Drive { node, input } => {
                let (kind, value) = match input {
                    Input::Set(strength) => (InputKind::Set, strength.get().into()),
                    Input::Toggle => (InputKind::Toggle, 0),
                    Input::Press => (InputKind::Press, 0),
                    Input::Touch(touched) => (InputKind::Touch, touched.into()),
                };
                let node = fitting(node.into(), DRIVE_NODE_BITS);
                let input = u64::from(kind) << INPUT_VALUE_BITS | value;
                (Opcode::Drive, input << DRIVE_NODE_BITS | node)
            }
            Request::Read(contents) => (Opcode::Read, contents.into()),
        };
        let mut request = [0; REQUEST_BYTES];
        request[0] = opcode as u8;
        request[1..].copy_from_slice(&argument.to_le_bytes()[..ARGUMENT_BYTES]);
        request
    }
}

impl Mode {
    pub fn counted(ticks: u64) -> anyhow::Result<Mode> {
        ensure!(
            ticks <= MAX_COUNT,
            "the board runs at most {MAX_COUNT} ticks at once"
        );
        Ok(Mode::Counted { ticks })
    }
}

impl Readout {
    pub fn decode(payload: &[u8]) -> Option<Readout> {
        let (ticks, state) = payload.split_first_chunk()?;
        Some(Readout {
            ticks: u64::from_le_bytes(*ticks),
            state: state.to_vec(),
        })
    }
}

impl Hello {
    const PAYLOAD_BYTES: usize = size_of::<u32>();

    pub fn decode(build_nonce: [u8; Self::PAYLOAD_BYTES]) -> Hello {
        Hello {
            build_nonce: u32::from_le_bytes(build_nonce),
        }
    }
}

/// CRC-16/CCITT-FALSE.
fn checksum(bytes: &[u8]) -> u16 {
    bytes.iter().fold(0xffff, |checksum, &byte| {
        (0..8).fold(checksum ^ u16::from(byte) << 8, |checksum, _| {
            checksum << 1 ^ if checksum & 0x8000 == 0 { 0 } else { 0x1021 }
        })
    })
}

pub fn frame(content: &[u8]) -> Vec<u8> {
    let mut frame = vec![FRAME_END];
    for &byte in content.iter().chain(&checksum(content).to_be_bytes()) {
        match byte {
            FRAME_END => frame.extend([ESCAPE, ESCAPED_END]),
            ESCAPE => frame.extend([ESCAPE, ESCAPED_ESCAPE]),
            byte => frame.push(byte),
        }
    }
    frame.push(FRAME_END);
    frame
}

fn unframe(stuffed: &[u8]) -> Result<Vec<u8>, AnswerError> {
    let mut content = Vec::with_capacity(stuffed.len());
    let mut bytes = stuffed.iter();
    while let Some(&byte) = bytes.next() {
        content.push(match byte {
            ESCAPE => match bytes.next() {
                Some(&ESCAPED_END) => FRAME_END,
                Some(&ESCAPED_ESCAPE) => ESCAPE,
                _ => return Err(AnswerError::Escape),
            },
            byte => byte,
        });
    }
    if content.len() <= CHECKSUM_BYTES {
        return Err(AnswerError::Truncated(content.len()));
    }
    if checksum(&content) != 0 {
        return Err(AnswerError::Checksum);
    }
    content.truncate(content.len() - CHECKSUM_BYTES);
    Ok(content)
}

pub fn payload(stuffed: &[u8]) -> Result<Vec<u8>, AnswerError> {
    let content = unframe(stuffed)?;
    let (&status, payload) = content.split_first().expect("unframe leaves the status");
    if status == STATUS_OK {
        return Ok(payload.to_vec());
    }
    let fault = Fault::ALL.iter().find(|&&fault| fault as u8 == status);
    Err(fault.map_or(AnswerError::UnknownStatus(status), |&fault| fault.into()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;
    use std::fmt::Debug;

    fn package_constants() -> BTreeMap<String, u64> {
        let literal = |text: &str| {
            let text = text.trim_end_matches([',', ';']);
            let Some((_, digits)) = text.split_once('\'') else {
                return text.parse().ok();
            };
            match digits.split_at_checked(1)? {
                ("d", digits) => digits.parse().ok(),
                ("h", digits) => u64::from_str_radix(digits, 16).ok(),
                _ => None,
            }
        };
        include_str!("../rtl/pkg.sv")
            .lines()
            .filter_map(|line| {
                let (declaration, value) = line.split_once(" = ")?;
                let name = declaration.split_whitespace().last()?;
                Some((name.to_owned(), literal(value)?))
            })
            .collect()
    }

    fn named<C: Debug + Copy + Into<u64>>(prefix: &str, codes: &[C]) -> Vec<(String, u64)> {
        let constant_name = |code: &C| {
            let mut name = prefix.to_owned();
            for letter in format!("{code:?}").chars() {
                if letter.is_uppercase() && name.len() != prefix.len() {
                    name.push('_');
                }
                name.push(letter.to_ascii_uppercase());
            }
            name
        };
        codes
            .iter()
            .map(|code| (constant_name(code), (*code).into()))
            .collect()
    }

    #[test]
    fn the_package_holds_the_constants() {
        let scalars = [
            ("FRAME_END", u64::from(FRAME_END)),
            ("ESCAPE", u64::from(ESCAPE)),
            ("ESCAPED_END", u64::from(ESCAPED_END)),
            ("ESCAPED_ESCAPE", u64::from(ESCAPED_ESCAPE)),
            ("OK", u64::from(STATUS_OK)),
            ("ARGUMENT_BYTES", ARGUMENT_BYTES as u64),
            ("COUNT_BITS", u64::from(COUNT_BITS)),
            (
                "PRIORITIES",
                mchprs_redstone::tick_queue::TickPriority::ALL.len() as u64,
            ),
        ];
        let host: BTreeMap<String, u64> = scalars
            .into_iter()
            .map(|(name, value)| (name.to_owned(), value))
            .chain(named("", Opcode::ALL))
            .chain(named("", Fault::ALL))
            .chain(named("", ModeKind::ALL))
            .chain(named("INPUT_", InputKind::ALL))
            .chain(named("READ_", Contents::ALL))
            .collect();
        assert_eq!(package_constants(), host);
    }
}
