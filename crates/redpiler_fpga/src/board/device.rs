use mchprs_world::TickRate;
use serialport::{ClearBuffer, SerialPort};
use std::io::{self, BufRead, BufReader, Write};
use std::time::Duration;

const BAUD: u32 = 4_000_000;
const CLOCK_HERTZ: f64 = 100_000_000.0;
const ANSWER_TIMEOUT: Duration = Duration::from_secs(2);
const FRAME_END: u8 = 0xc0;
const ESCAPE: u8 = 0xdb;
const ESCAPED_END: u8 = 0xdc;
const ESCAPED_ESCAPE: u8 = 0xdd;
const HEADER_LENGTH: usize = 9;
const CHECKSUM_LENGTH: usize = 2;

#[derive(Debug, thiserror::Error)]
pub(crate) enum DeviceError {
    #[error("could not open the FPGA serial port {port}")]
    Open {
        port: String,
        #[source]
        source: serialport::Error,
    },
    #[error("FPGA serial port failed")]
    Port(#[from] serialport::Error),
    #[error("FPGA serial exchange failed")]
    Io(#[from] io::Error),
    #[error("FPGA did not answer in time")]
    TimedOut,
    #[error("FPGA answer is corrupt")]
    Corrupt,
    #[error("FPGA rejected {command:?} with fault {fault}")]
    Rejected { command: Command, fault: u8 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Data {
    Outputs,
    States,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Command {
    Status,
    Input { node: u32, powered: bool },
    Run(TickRate),
    Read(Data),
}

impl Command {
    fn encode(self) -> [u8; 9] {
        let (code, argument) = match self {
            Self::Status => (0, 0),
            Self::Input { node, powered } => (1, u64::from(node) | u64::from(powered) << 31),
            Self::Run(TickRate::Paused) => (2, 0),
            Self::Run(TickRate::Unlimited) => (3, 0),
            Self::Run(TickRate::Ticks(ticks)) => (4, ticks),
            Self::Run(TickRate::PerSecond(per_second)) => {
                (5, (CLOCK_HERTZ / per_second).round().max(1.0) as u64)
            }
            Self::Read(Data::Outputs) => (6, 0),
            Self::Read(Data::States) => (7, 0),
        };
        let mut bytes = [code; 9];
        bytes[1..].copy_from_slice(&argument.to_le_bytes());
        bytes
    }
}

#[derive(Debug, PartialEq, Eq)]
pub(super) struct Answer {
    pub(super) ticks: u64,
    pub(super) payload: Vec<u8>,
}

pub(super) struct Board {
    port: BufReader<Box<dyn SerialPort>>,
}

impl Board {
    pub(super) fn open(port: &str) -> Result<Board, DeviceError> {
        let opened = serialport::new(port, BAUD)
            .timeout(ANSWER_TIMEOUT)
            .open()
            .map_err(|source| DeviceError::Open {
                port: port.to_owned(),
                source,
            })?;
        Ok(Board {
            port: BufReader::new(opened),
        })
    }

    pub(super) fn discard_received(&mut self) -> Result<(), DeviceError> {
        let buffered = self.port.buffer().len();
        self.port.consume(buffered);
        self.port.get_mut().clear(ClearBuffer::All)?;
        Ok(())
    }

    pub(super) fn request(&mut self, command: Command) -> Result<Answer, DeviceError> {
        self.port.get_mut().write_all(&frame(&command.encode()))?;
        let mut stuffed = Vec::new();
        while stuffed.len() < 2 {
            stuffed.clear();
            self.port
                .read_until(FRAME_END, &mut stuffed)
                .map_err(|error| match error.kind() {
                    io::ErrorKind::TimedOut => DeviceError::TimedOut,
                    _ => error.into(),
                })?;
        }
        stuffed.pop();
        decode_answer(command, &stuffed)
    }
}

fn frame(content: &[u8]) -> Vec<u8> {
    let mut frame = vec![FRAME_END];
    for &byte in content.iter().chain(&crc16(content).to_be_bytes()) {
        match byte {
            FRAME_END => frame.extend([ESCAPE, ESCAPED_END]),
            ESCAPE => frame.extend([ESCAPE, ESCAPED_ESCAPE]),
            byte => frame.push(byte),
        }
    }
    frame.push(FRAME_END);
    frame
}

fn decode_answer(command: Command, stuffed: &[u8]) -> Result<Answer, DeviceError> {
    let mut content = Vec::with_capacity(stuffed.len());
    let mut bytes = stuffed.iter();
    while let Some(&byte) = bytes.next() {
        content.push(match byte {
            ESCAPE => match bytes.next() {
                Some(&ESCAPED_END) => FRAME_END,
                Some(&ESCAPED_ESCAPE) => ESCAPE,
                _ => return Err(DeviceError::Corrupt),
            },
            byte => byte,
        });
    }
    if content.len() < HEADER_LENGTH + CHECKSUM_LENGTH || crc16(&content) != 0 {
        return Err(DeviceError::Corrupt);
    }
    if content[0] != 0 {
        return Err(DeviceError::Rejected {
            command,
            fault: content[0],
        });
    }
    Ok(Answer {
        ticks: u64::from_le_bytes(content[1..HEADER_LENGTH].try_into().expect("eight bytes")),
        payload: content[HEADER_LENGTH..content.len() - CHECKSUM_LENGTH].to_vec(),
    })
}

fn crc16(bytes: &[u8]) -> u16 {
    bytes.iter().fold(0xffff, |mut checksum, byte| {
        checksum ^= u16::from(*byte) << 8;
        for _ in 0..8 {
            checksum = if checksum & 0x8000 == 0 {
                checksum << 1
            } else {
                (checksum << 1) ^ 0x1021
            };
        }
        checksum
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn answer(fault: u8, ticks: u64, payload: &[u8]) -> Vec<u8> {
        let mut content = vec![fault];
        content.extend(ticks.to_le_bytes());
        content.extend(payload);
        let framed = frame(&content);
        framed[1..framed.len() - 1].to_vec()
    }

    #[test]
    fn checksum_is_crc16_ccitt_false() {
        assert_eq!(crc16(b"123456789"), 0x29b1);
    }

    #[test]
    fn commands_carry_a_code_and_argument() {
        let input = Command::Input {
            node: 0x0102_0304,
            powered: true,
        };
        assert_eq!(input.encode(), [1, 0x04, 0x03, 0x02, 0x81, 0, 0, 0, 0]);
        let ticks = Command::Run(TickRate::Ticks(300)).encode();
        assert_eq!(ticks, [4, 44, 1, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn tick_rates_become_clocks_per_tick() {
        let clocks = |per_second| {
            let bytes = Command::Run(TickRate::PerSecond(per_second)).encode();
            (bytes[0], u64::from_le_bytes(bytes[1..].try_into().unwrap()))
        };
        assert_eq!(clocks(20.0), (5, 5_000_000));
        assert_eq!(clocks(0.1), (5, 1_000_000_000));
        assert_eq!(clocks(1e9), (5, 1));
    }

    #[test]
    fn answers_survive_stuffing() {
        let payload = [FRAME_END, ESCAPE, 0xad];
        assert_eq!(
            decode_answer(Command::Status, &answer(0, 42, &payload)).unwrap(),
            Answer {
                ticks: 42,
                payload: payload.to_vec(),
            }
        );
    }

    #[test]
    fn faults_and_corruption_are_errors() {
        assert!(matches!(
            decode_answer(Command::Status, &answer(5, 0, &[])),
            Err(DeviceError::Rejected { fault: 5, .. })
        ));
        let mut corrupted = answer(0, 42, &[1]);
        corrupted[3] ^= 1;
        assert!(matches!(
            decode_answer(Command::Status, &corrupted),
            Err(DeviceError::Corrupt)
        ));
    }
}
