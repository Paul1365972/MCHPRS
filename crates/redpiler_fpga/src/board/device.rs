use crate::circuit::Circuit;
use std::io;
use std::time::{Duration, Instant};

const TIMEOUT: Duration = Duration::from_secs(2);
const BAUD: u32 = 1_000_000;
const UART_BITS_PER_BYTE: u64 = 10;
const PROTOCOL_VERSION: u16 = 1;
const REQUEST_SYNCHRONIZATION: [u8; 2] = [0xa5, 0x5a];
const RESPONSE_SYNCHRONIZATION: [u8; 2] = [0x5a, 0xa5];
const REQUEST_LENGTH: usize = 16;
const RESPONSE_HEADER_LENGTH: usize = 16;
const CHECKSUM_LENGTH: usize = 2;
const REJECTED_OPCODE: u8 = 0xff;

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
    #[error("FPGA response synchronization mismatch")]
    Synchronization,
    #[error("FPGA response sequence mismatch")]
    Sequence,
    #[error("FPGA response opcode rejected or mismatched")]
    Opcode,
    #[error("FPGA response node count mismatch")]
    NodeCount,
    #[error("FPGA response CRC16 mismatch")]
    Checksum,
}

#[derive(Clone, Copy, Debug)]
pub(super) enum Command {
    Snapshot,
    Input { index: u32, powered: bool },
    Identity,
    Status,
    RunUntil(u64),
    Outputs,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Payload {
    None,
    States,
    Outputs,
}

impl Command {
    fn opcode(self) -> u8 {
        match self {
            Self::Snapshot => 0,
            Self::Input { .. } => 2,
            Self::Identity => 4,
            Self::Status => 7,
            Self::RunUntil(_) => 8,
            Self::Outputs => 9,
        }
    }

    fn argument(self) -> u64 {
        match self {
            Self::Input { index, powered } => u64::from(index) | u64::from(powered) << 32,
            Self::RunUntil(tick) => tick,
            Self::Snapshot | Self::Identity | Self::Status | Self::Outputs => 0,
        }
    }

    fn payload(self) -> Payload {
        match self {
            Self::Snapshot | Self::Input { .. } | Self::Identity => Payload::States,
            Self::Status | Self::RunUntil(_) => Payload::None,
            Self::Outputs => Payload::Outputs,
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub(super) struct Response {
    pub(super) ticks: u64,
    pub(super) data: Vec<u8>,
}

pub(super) struct Device {
    port: Option<Box<dyn serialport::SerialPort>>,
    node_count: usize,
    output_count: usize,
    state_bytes: usize,
    output_bytes: usize,
    sequence: u8,
}

impl Device {
    pub(super) fn serial(
        port: &str,
        circuit: &Circuit,
        state_bytes: usize,
        output_bytes: usize,
    ) -> Result<Self, DeviceError> {
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
            node_count: circuit.nodes().len(),
            output_count: circuit.io_nodes().len(),
            state_bytes,
            output_bytes,
            sequence: 0,
        })
    }

    pub(super) fn execute(&mut self, command: Command) -> Result<Response, DeviceError> {
        let port = self.port.as_mut().ok_or(DeviceError::Stopped)?;
        let (count, payload_bytes) = match command.payload() {
            Payload::None => (self.node_count, 0),
            Payload::States => (self.node_count, self.state_bytes),
            Payload::Outputs => (self.output_count, self.output_bytes),
        };
        let request = request_frame(self.sequence, command);
        let mut response = vec![0; RESPONSE_HEADER_LENGTH + payload_bytes + CHECKSUM_LENGTH];
        let result = serial_exchange(port.as_mut(), &request, &mut response)
            .and_then(|()| decode_response(&response, self.sequence, command, count));
        match result {
            Ok(response) => {
                self.sequence = self.sequence.wrapping_add(1);
                Ok(response)
            }
            Err(error) => {
                self.port = None;
                Err(error)
            }
        }
    }
}

fn serial_exchange(
    port: &mut dyn serialport::SerialPort,
    request: &[u8; REQUEST_LENGTH],
    response: &mut [u8],
) -> Result<(), DeviceError> {
    let deadline = Instant::now() + exchange_timeout(response.len());
    let mut remaining = request.as_slice();
    while !remaining.is_empty() {
        port.set_timeout(time_remaining(deadline)?)?;
        match port.write(remaining) {
            Ok(0) => return Err(DeviceError::WriteStopped),
            Ok(count) => remaining = &remaining[count..],
            Err(error) if error.kind() == io::ErrorKind::Interrupted => {}
            Err(error) => return Err(error.into()),
        }
    }
    let mut remaining = response;
    while !remaining.is_empty() {
        port.set_timeout(time_remaining(deadline)?)?;
        match port.read(remaining) {
            Ok(0) => return Err(DeviceError::ResponseEnded),
            Ok(count) => remaining = &mut remaining[count..],
            Err(error) if error.kind() == io::ErrorKind::Interrupted => {}
            Err(error) => return Err(error.into()),
        }
    }
    Ok(())
}

fn time_remaining(deadline: Instant) -> Result<Duration, DeviceError> {
    deadline
        .checked_duration_since(Instant::now())
        .ok_or(DeviceError::TimedOut)
}

fn request_frame(sequence: u8, command: Command) -> [u8; REQUEST_LENGTH] {
    let mut frame = [0; REQUEST_LENGTH];
    frame[..2].copy_from_slice(&REQUEST_SYNCHRONIZATION);
    frame[2] = sequence;
    frame[3] = command.opcode();
    frame[4..12].copy_from_slice(&command.argument().to_le_bytes());
    frame[12..14].copy_from_slice(&PROTOCOL_VERSION.to_le_bytes());
    let checksum = crc16(&frame[2..14]);
    frame[14..].copy_from_slice(&checksum.to_le_bytes());
    frame
}

fn decode_response(
    response: &[u8],
    sequence: u8,
    command: Command,
    node_count: usize,
) -> Result<Response, DeviceError> {
    let (checked, checksum) = response.split_at(response.len() - CHECKSUM_LENGTH);
    let (header, data) = checked.split_at(RESPONSE_HEADER_LENGTH);
    if header[..2] != RESPONSE_SYNCHRONIZATION {
        return Err(DeviceError::Synchronization);
    }
    if header[2] != sequence {
        return Err(DeviceError::Sequence);
    }
    if header[3] == REJECTED_OPCODE || header[3] != command.opcode() {
        return Err(DeviceError::Opcode);
    }
    let count = u32::from_le_bytes(header[12..16].try_into().expect("four count bytes"));
    if count as usize != node_count {
        return Err(DeviceError::NodeCount);
    }
    let checksum = u16::from_le_bytes(checksum.try_into().expect("two checksum bytes"));
    if crc16(&checked[2..]) != checksum {
        return Err(DeviceError::Checksum);
    }
    Ok(Response {
        ticks: u64::from_le_bytes(header[4..12].try_into().expect("eight tick bytes")),
        data: data.to_vec(),
    })
}

fn exchange_timeout(response_length: usize) -> Duration {
    let wire_bits = (response_length + REQUEST_LENGTH) as u64 * UART_BITS_PER_BYTE;
    let baud = u64::from(BAUD);
    TIMEOUT
        + Duration::from_secs(wire_bits / baud)
        + Duration::from_nanos(((wire_bits % baud) * 1_000_000_000).div_ceil(baud))
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

    fn response(sequence: u8, opcode: u8, ticks: u64, count: u32, data: &[u8]) -> Vec<u8> {
        let mut response = RESPONSE_SYNCHRONIZATION.to_vec();
        response.extend([sequence, opcode]);
        response.extend(ticks.to_le_bytes());
        response.extend(count.to_le_bytes());
        response.extend(data);
        let checksum = crc16(&response[2..]);
        response.extend(checksum.to_le_bytes());
        response
    }

    #[test]
    fn checksum_is_crc16_ccitt_false() {
        assert_eq!(crc16(b"123456789"), 0x29b1);
    }

    #[test]
    fn requests_carry_opcode_argument_version_and_checksum() {
        let frame = request_frame(
            7,
            Command::Input {
                index: 0x0403_0201,
                powered: true,
            },
        );
        assert_eq!(
            frame[..14],
            [0xa5, 0x5a, 7, 2, 1, 2, 3, 4, 1, 0, 0, 0, 1, 0]
        );
        assert_eq!(frame[14..], crc16(&frame[2..14]).to_le_bytes());
        let frame = request_frame(0, Command::RunUntil(0x0102));
        assert_eq!(frame[3..12], [8, 2, 1, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn only_status_and_run_until_responses_omit_the_payload() {
        let commands = [
            (Command::Snapshot, Payload::States),
            (
                Command::Input {
                    index: 0,
                    powered: false,
                },
                Payload::States,
            ),
            (Command::Identity, Payload::States),
            (Command::Status, Payload::None),
            (Command::RunUntil(0), Payload::None),
            (Command::Outputs, Payload::Outputs),
        ];
        for (command, payload) in commands {
            assert_eq!(command.payload(), payload, "{command:?}");
        }
    }

    #[test]
    fn responses_decode_ticks_and_data() {
        let bytes = response(3, 0, 42, 5, &[0xde, 0xad]);
        assert_eq!(
            decode_response(&bytes, 3, Command::Snapshot, 5).unwrap(),
            Response {
                ticks: 42,
                data: vec![0xde, 0xad],
            }
        );
    }

    #[test]
    fn corrupt_responses_are_rejected() {
        let valid = response(3, 9, 42, 5, &[0xde, 0xad]);
        let decode = |bytes: &[u8]| decode_response(bytes, 3, Command::Outputs, 5);
        let mut unsynchronized = valid.clone();
        unsynchronized[0] = 0;
        assert!(matches!(
            decode(&unsynchronized),
            Err(DeviceError::Synchronization)
        ));
        assert!(matches!(
            decode(&response(4, 9, 42, 5, &[0xde, 0xad])),
            Err(DeviceError::Sequence)
        ));
        assert!(matches!(
            decode(&response(3, REJECTED_OPCODE, 42, 5, &[0xde, 0xad])),
            Err(DeviceError::Opcode)
        ));
        assert!(matches!(
            decode(&response(3, 9, 42, 6, &[0xde, 0xad])),
            Err(DeviceError::NodeCount)
        ));
        let mut corrupted = valid.clone();
        corrupted[16] ^= 1;
        assert!(matches!(decode(&corrupted), Err(DeviceError::Checksum)));
    }
}
