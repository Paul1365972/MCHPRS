use crate::rtl::MAX_INPUTS_PER_REQUEST;
use std::io;
use std::time::{Duration, Instant};

const TIMEOUT: Duration = Duration::from_secs(2);
const BAUD: u32 = 4_000_000;
const UART_BITS_PER_BYTE: u64 = 10;
const MAX_TICKS_PER_SECOND: u64 = 25_000_000;
const PROTOCOL_VERSION: u8 = 2;
const FRAME_END: u8 = 0xc0;
const ESCAPE: u8 = 0xdb;
const ESCAPED_END: u8 = 0xdc;
const ESCAPED_ESCAPE: u8 = 0xdd;
const IDENTIFY: u8 = 1;
const ADVANCE: u8 = 2;
const HEADER_LENGTH: usize = 10;
const CHECKSUM_LENGTH: usize = 2;
const IDENTITY_LENGTH: usize = 27;

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
    #[error("FPGA device stopped after a previous failure")]
    Stopped,
    #[error("FPGA serial write stopped")]
    WriteStopped,
    #[error("FPGA serial response ended")]
    ResponseEnded,
    #[error("FPGA serial request timed out")]
    TimedOut,
    #[error("FPGA response has an invalid escape")]
    Escape,
    #[error("FPGA response CRC16 mismatch")]
    Checksum,
    #[error("FPGA response sequence mismatch")]
    Sequence,
    #[error("FPGA rejected the request")]
    Rejected(#[source] Rejection),
    #[error("FPGA response has {actual} payload bytes instead of {expected}")]
    Length { actual: usize, expected: usize },
    #[error("FPGA speaks protocol version {0} instead of {PROTOCOL_VERSION}")]
    Version(u8),
    #[error("FPGA circuit {actual:?} is not the compiled circuit {expected:?}")]
    Identity {
        expected: Identity,
        actual: Identity,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub(crate) enum Rejection {
    #[error("unknown command")]
    Command,
    #[error("an input node is out of range")]
    Input,
    #[error("the target tick has passed")]
    Target,
    #[error("the report is not available in this build")]
    Report,
    #[error("unknown status {0}")]
    Unknown(u8),
}

impl Rejection {
    fn from_status(status: u8) -> Self {
        match status {
            4 => Self::Command,
            5 => Self::Input,
            6 => Self::Target,
            7 => Self::Report,
            status => Self::Unknown(status),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Identity {
    pub(crate) circuit_id: u64,
    pub(crate) node_count: u32,
    pub(crate) output_count: u32,
    pub(crate) state_bytes: u32,
    pub(crate) output_bytes: u32,
    pub(crate) max_inputs: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Report {
    Nothing,
    Outputs,
    States,
}

impl Report {
    fn code(self) -> u8 {
        match self {
            Self::Nothing => 0,
            Self::Outputs => 1,
            Self::States => 2,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Input {
    pub(super) node: u32,
    pub(super) powered: bool,
}

impl Input {
    fn word(self) -> u32 {
        self.node | u32::from(self.powered) << 31
    }
}

#[derive(Debug, PartialEq, Eq)]
pub(super) struct Response {
    pub(super) ticks: u64,
    pub(super) data: Vec<u8>,
}

pub(super) struct Device {
    port: Option<Box<dyn serialport::SerialPort>>,
    identity: Identity,
    sequence: u8,
    ticks: u64,
}

impl Device {
    pub(super) fn serial(port: &str, identity: Identity) -> Result<Self, DeviceError> {
        let mut serial = serialport::new(port, BAUD)
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .flow_control(serialport::FlowControl::None)
            .timeout(TIMEOUT)
            .open()
            .map_err(|source| DeviceError::Open {
                port: port.to_owned(),
                source,
            })?;
        serial.flush()?;
        serial.clear(serialport::ClearBuffer::All)?;
        Ok(Self {
            port: Some(serial),
            identity,
            sequence: 0,
            ticks: 0,
        })
    }

    pub(super) fn identify(&mut self) -> Result<(), DeviceError> {
        let data = self
            .exchange(&[IDENTIFY], IDENTITY_LENGTH, self.ticks)?
            .data;
        if data[0] != PROTOCOL_VERSION {
            return Err(DeviceError::Version(data[0]));
        }
        let word = |offset: usize| {
            u32::from_le_bytes(data[offset..offset + 4].try_into().expect("four bytes"))
        };
        let actual = Identity {
            circuit_id: u64::from_le_bytes(data[1..9].try_into().expect("eight bytes")),
            node_count: word(9),
            output_count: word(13),
            state_bytes: word(17),
            output_bytes: word(21),
            max_inputs: u16::from_le_bytes(data[25..27].try_into().expect("two bytes")),
        };
        if actual != self.identity {
            return Err(DeviceError::Identity {
                expected: self.identity,
                actual,
            });
        }
        Ok(())
    }

    pub(super) fn advance(
        &mut self,
        inputs: &[Input],
        target: u64,
        report: Report,
    ) -> Result<Response, DeviceError> {
        let last_chunk = inputs.len().saturating_sub(1) / MAX_INPUTS_PER_REQUEST;
        let (earlier, last) = inputs.split_at(last_chunk * MAX_INPUTS_PER_REQUEST);
        for chunk in earlier.chunks(MAX_INPUTS_PER_REQUEST) {
            self.advance_once(chunk, self.ticks, Report::Nothing)?;
        }
        self.advance_once(last, target, report)
    }

    fn advance_once(
        &mut self,
        inputs: &[Input],
        target: u64,
        report: Report,
    ) -> Result<Response, DeviceError> {
        let mut request = vec![ADVANCE];
        request.extend(target.to_le_bytes());
        request.push(report.code());
        for input in inputs {
            request.extend(input.word().to_le_bytes());
        }
        let payload = match report {
            Report::Nothing => 0,
            Report::Outputs => self.identity.output_bytes as usize,
            Report::States => self.identity.state_bytes as usize,
        };
        self.exchange(&request, payload, target)
    }

    fn exchange(
        &mut self,
        command: &[u8],
        payload: usize,
        target: u64,
    ) -> Result<Response, DeviceError> {
        let port = self.port.as_mut().ok_or(DeviceError::Stopped)?;
        let mut content = vec![self.sequence];
        content.extend(command);
        let request = frame(&content);
        let run = Duration::from_secs_f64(
            target.saturating_sub(self.ticks) as f64 / MAX_TICKS_PER_SECOND as f64,
        );
        let wire = wire_time(request.len() + 2 * (HEADER_LENGTH + payload + CHECKSUM_LENGTH));
        let deadline = Instant::now() + TIMEOUT + run + wire;
        let result = serial_exchange(port.as_mut(), &request, deadline)
            .and_then(|frame| decode_response(&frame, self.sequence, payload));
        match result {
            Ok(response) => {
                self.sequence = self.sequence.wrapping_add(1);
                self.ticks = response.ticks;
                Ok(response)
            }
            Err(error) => {
                self.port = None;
                Err(error)
            }
        }
    }
}

fn frame(content: &[u8]) -> Vec<u8> {
    let checksum = crc16(content);
    let mut frame = vec![FRAME_END];
    for &byte in content.iter().chain(&checksum.to_be_bytes()) {
        match byte {
            FRAME_END => frame.extend([ESCAPE, ESCAPED_END]),
            ESCAPE => frame.extend([ESCAPE, ESCAPED_ESCAPE]),
            byte => frame.push(byte),
        }
    }
    frame.push(FRAME_END);
    frame
}

fn unstuff(stuffed: &[u8]) -> Result<Vec<u8>, DeviceError> {
    let mut content = Vec::with_capacity(stuffed.len());
    let mut bytes = stuffed.iter();
    while let Some(&byte) = bytes.next() {
        content.push(if byte == ESCAPE {
            match bytes.next() {
                Some(&ESCAPED_END) => FRAME_END,
                Some(&ESCAPED_ESCAPE) => ESCAPE,
                _ => return Err(DeviceError::Escape),
            }
        } else {
            byte
        });
    }
    Ok(content)
}

fn serial_exchange(
    port: &mut dyn serialport::SerialPort,
    request: &[u8],
    deadline: Instant,
) -> Result<Vec<u8>, DeviceError> {
    let mut remaining = request;
    while !remaining.is_empty() {
        port.set_timeout(time_remaining(deadline)?)?;
        match port.write(remaining) {
            Ok(0) => return Err(DeviceError::WriteStopped),
            Ok(count) => remaining = &remaining[count..],
            Err(error) if error.kind() == io::ErrorKind::Interrupted => {}
            Err(error) => return Err(error.into()),
        }
    }
    let mut stuffed = Vec::new();
    let mut buffer = [0; 4096];
    loop {
        port.set_timeout(time_remaining(deadline)?)?;
        let count = match port.read(&mut buffer) {
            Ok(0) => return Err(DeviceError::ResponseEnded),
            Ok(count) => count,
            Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
            Err(error) => return Err(error.into()),
        };
        for &byte in &buffer[..count] {
            match byte {
                FRAME_END if stuffed.is_empty() => {}
                FRAME_END => return Ok(stuffed),
                byte => stuffed.push(byte),
            }
        }
    }
}

fn time_remaining(deadline: Instant) -> Result<Duration, DeviceError> {
    deadline
        .checked_duration_since(Instant::now())
        .ok_or(DeviceError::TimedOut)
}

fn decode_response(stuffed: &[u8], sequence: u8, payload: usize) -> Result<Response, DeviceError> {
    let content = unstuff(stuffed)?;
    if content.len() < HEADER_LENGTH + CHECKSUM_LENGTH || crc16(&content) != 0 {
        return Err(DeviceError::Checksum);
    }
    if content[0] != sequence {
        return Err(DeviceError::Sequence);
    }
    if content[1] != 0 {
        return Err(DeviceError::Rejected(Rejection::from_status(content[1])));
    }
    let data = &content[HEADER_LENGTH..content.len() - CHECKSUM_LENGTH];
    if data.len() != payload {
        return Err(DeviceError::Length {
            actual: data.len(),
            expected: payload,
        });
    }
    Ok(Response {
        ticks: u64::from_le_bytes(content[2..HEADER_LENGTH].try_into().unwrap()),
        data: data.to_vec(),
    })
}

fn wire_time(bytes: usize) -> Duration {
    let bits = bytes as u64 * UART_BITS_PER_BYTE;
    Duration::from_nanos((bits * 1_000_000_000).div_ceil(u64::from(BAUD)))
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

    fn response(sequence: u8, status: u8, ticks: u64, data: &[u8]) -> Vec<u8> {
        let mut content = vec![sequence, status];
        content.extend(ticks.to_le_bytes());
        content.extend(data);
        let framed = frame(&content);
        framed[1..framed.len() - 1].to_vec()
    }

    #[test]
    fn checksum_is_crc16_ccitt_false() {
        assert_eq!(crc16(b"123456789"), 0x29b1);
    }

    #[test]
    fn frames_end_with_a_checksum_that_leaves_no_remainder() {
        let content = [7, ADVANCE, 1, 2, 3];
        let framed = frame(&content);
        assert_eq!(
            (framed[0], framed[framed.len() - 1]),
            (FRAME_END, FRAME_END)
        );
        let unstuffed = unstuff(&framed[1..framed.len() - 1]).unwrap();
        assert_eq!(unstuffed[..content.len()], content);
        assert_eq!(crc16(&unstuffed), 0);
    }

    #[test]
    fn stuffing_hides_frame_ends_and_round_trips() {
        let content = [FRAME_END, ESCAPE, 0, ESCAPED_END, FRAME_END];
        let framed = frame(&content);
        assert!(!framed[1..framed.len() - 1].contains(&FRAME_END));
        let unstuffed = unstuff(&framed[1..framed.len() - 1]).unwrap();
        assert_eq!(unstuffed[..content.len()], content);
    }

    #[test]
    fn inputs_pack_the_node_and_the_powered_bit() {
        let input = Input {
            node: 0x0102_0304,
            powered: true,
        };
        assert_eq!(input.word(), 0x8102_0304);
    }

    #[test]
    fn responses_decode_ticks_and_data() {
        let bytes = response(3, 0, 42, &[FRAME_END, 0xad]);
        assert_eq!(
            decode_response(&bytes, 3, 2).unwrap(),
            Response {
                ticks: 42,
                data: vec![FRAME_END, 0xad],
            }
        );
    }

    #[test]
    fn corrupt_and_rejected_responses_are_errors() {
        assert!(matches!(
            decode_response(&response(4, 0, 42, &[1]), 3, 1),
            Err(DeviceError::Sequence)
        ));
        assert!(matches!(
            decode_response(&response(3, 6, 42, &[]), 3, 0),
            Err(DeviceError::Rejected(Rejection::Target))
        ));
        assert!(matches!(
            decode_response(&response(3, 0, 42, &[1]), 3, 2),
            Err(DeviceError::Length {
                actual: 1,
                expected: 2
            })
        ));
        let mut corrupted = response(3, 0, 42, &[1]);
        corrupted[2] ^= 1;
        assert!(matches!(
            decode_response(&corrupted, 3, 1),
            Err(DeviceError::Checksum)
        ));
        assert!(matches!(
            decode_response(&[ESCAPE, 0], 3, 0),
            Err(DeviceError::Escape)
        ));
    }
}
