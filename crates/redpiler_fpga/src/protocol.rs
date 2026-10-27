//! The wire constants are shared with `rtl/pkg.sv`.
//! Replies carry a status byte followed by a little-endian payload.
//! `HELLO` returns the build nonce and PLL lock flag.
//! `PLL_READ` returns a 16-bit register.
//! `RUN`, `DRIVE` and `READ` return the tick count.
//! `DRIVE` and `READ` append the requested state padded to whole bytes.

use anyhow::ensure;
use mchprs_redstone::input::Input;
use strum::FromRepr;

// The framing bytes of SLIP (RFC 1055).
pub const FRAME_END: u8 = 0xc0;
const ESCAPE: u8 = 0xdb;
const ESCAPED_END: u8 = 0xdc;
const ESCAPED_ESCAPE: u8 = 0xdd;
const STATUS_OK: u8 = 0;
const ARGUMENT_BYTES: usize = 6;
const REQUEST_BYTES: usize = 1 + ARGUMENT_BYTES;
const COUNT_BITS: u32 = 40;
pub const MAX_COUNT: u64 = (1 << COUNT_BITS) - 1;
pub const DRIVE_NODE_BITS: u32 = 24;
const PLL_DATA_BITS: u32 = 16;
const PLL_ADDRESS_BITS: u32 = 7;
const INPUT_VALUE_BITS: u32 = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Opcode {
    Hello = 0,
    Run = 1,
    Drive = 2,
    Read = 3,
    PllWrite = 4,
    PllRead = 5,
    ReleaseClock = 6,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ModeKind {
    Paused = 0,
    Unlimited = 1,
    Counted = 2,
    Periodic = 3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InputKind {
    Set = 0,
    Toggle = 1,
    Press = 2,
    Touch = 3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Contents {
    Shown = 0,
    All = 1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error, FromRepr)]
#[repr(u8)]
pub enum Fault {
    #[error("the request has the wrong length")]
    BadLength = 1,
    #[error("the request has a broken escape sequence")]
    BadEscape = 2,
    #[error("the opcode is unknown")]
    UnknownOpcode = 3,
    #[error("the circuit clock is stopped")]
    ClockStopped = 4,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Request {
    Hello,
    Run(Mode),
    Drive { node: u32, input: Input },
    Read(Contents),
    PllWrite { address: u8, data: u16 },
    PllRead { address: u8 },
    ReleaseClock,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Paused,
    Unlimited,
    Counted { ticks: u64 },
    Periodic { clocks_per_tick: u64 },
}

/// A reply that includes state clears the note-block events it reports.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Readout {
    pub ticks: u64,
    pub state: Vec<u8>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Hello {
    pub build_nonce: u32,
    pub pll_locked: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum AnswerError {
    #[error("the answer has a broken escape sequence")]
    Escape,
    #[error("the answer ends after {0} bytes")]
    Truncated(usize),
    #[error("the board refused the request: {0}")]
    Fault(#[from] Fault),
    #[error("the answer has the unknown status {0}")]
    UnknownStatus(u8),
}

impl Request {
    /// Packs the fields of `pkg::run_t`, `pkg::drive_t` and `pkg::pll_access_t`
    /// from most to least significant.
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
                (Opcode::Run, (kind as u64) << COUNT_BITS | count)
            }
            Request::Drive { node, input } => {
                let (kind, value) = match input {
                    Input::Set(strength) => (InputKind::Set, strength.get().into()),
                    Input::Toggle => (InputKind::Toggle, 0),
                    Input::Press => (InputKind::Press, 0),
                    Input::Touch(touched) => (InputKind::Touch, touched.into()),
                };
                let node = fitting(node.into(), DRIVE_NODE_BITS);
                let input = (kind as u64) << INPUT_VALUE_BITS | value;
                (Opcode::Drive, input << DRIVE_NODE_BITS | node)
            }
            Request::Read(contents) => (Opcode::Read, contents as u64),
            Request::PllWrite { address, data } => {
                let address = fitting(address.into(), PLL_ADDRESS_BITS);
                (Opcode::PllWrite, address << PLL_DATA_BITS | u64::from(data))
            }
            Request::PllRead { address } => {
                let address = fitting(address.into(), PLL_ADDRESS_BITS);
                (Opcode::PllRead, address << PLL_DATA_BITS)
            }
            Request::ReleaseClock => (Opcode::ReleaseClock, 0),
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
    const PAYLOAD_BYTES: usize = size_of::<u32>() + size_of::<u8>();

    pub fn decode(payload: [u8; Self::PAYLOAD_BYTES]) -> Hello {
        let [build_nonce @ .., flags] = payload;
        Hello {
            build_nonce: u32::from_le_bytes(build_nonce),
            pll_locked: flags & 1 != 0,
        }
    }
}

pub fn frame(content: &[u8]) -> Vec<u8> {
    let mut frame = vec![FRAME_END];
    for &byte in content {
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
    if content.is_empty() {
        return Err(AnswerError::Truncated(0));
    }
    Ok(content)
}

pub fn payload(stuffed: &[u8]) -> Result<Vec<u8>, AnswerError> {
    let content = unframe(stuffed)?;
    let (&status, payload) = content.split_first().expect("unframe leaves the status");
    if status == STATUS_OK {
        return Ok(payload.to_vec());
    }
    let fault = Fault::from_repr(status);
    Err(fault.map_or(AnswerError::UnknownStatus(status), AnswerError::Fault))
}
