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
    ) -> io::Result<Self> {
        let mut port = serialport::new(port, BAUD)
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .flow_control(serialport::FlowControl::None)
            .timeout(TIMEOUT)
            .open()
            .map_err(io::Error::other)?;
        port.flush()?;
        port.clear(serialport::ClearBuffer::All)
            .map_err(io::Error::other)?;
        Ok(Self {
            port: Some(port),
            node_count: circuit.nodes().len(),
            output_count: circuit.io_nodes().len(),
            state_bytes,
            output_bytes,
            sequence: 0,
        })
    }

    pub(super) fn execute(&mut self, command: Command) -> io::Result<Response> {
        let port = self.port.as_mut().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::BrokenPipe,
                "FPGA device stopped after a previous failure",
            )
        })?;
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
) -> io::Result<()> {
    let deadline = Instant::now() + exchange_timeout(response.len());
    let mut remaining = request.as_slice();
    while !remaining.is_empty() {
        port.set_timeout(time_remaining(deadline)?)
            .map_err(io::Error::other)?;
        match port.write(remaining) {
            Ok(0) => {
                return Err(io::Error::new(
                    io::ErrorKind::WriteZero,
                    "FPGA serial write stopped",
                ));
            }
            Ok(count) => remaining = &remaining[count..],
            Err(error) if error.kind() == io::ErrorKind::Interrupted => {}
            Err(error) => return Err(error),
        }
    }
    let mut remaining = response;
    while !remaining.is_empty() {
        port.set_timeout(time_remaining(deadline)?)
            .map_err(io::Error::other)?;
        match port.read(remaining) {
            Ok(0) => {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "FPGA serial response ended",
                ));
            }
            Ok(count) => remaining = &mut remaining[count..],
            Err(error) if error.kind() == io::ErrorKind::Interrupted => {}
            Err(error) => return Err(error),
        }
    }
    Ok(())
}

fn time_remaining(deadline: Instant) -> io::Result<Duration> {
    deadline
        .checked_duration_since(Instant::now())
        .ok_or_else(|| io::Error::new(io::ErrorKind::TimedOut, "FPGA serial request timed out"))
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
) -> io::Result<Response> {
    let invalid = |message| io::Error::new(io::ErrorKind::InvalidData, message);
    let (checked, checksum) = response.split_at(response.len() - CHECKSUM_LENGTH);
    let (header, data) = checked.split_at(RESPONSE_HEADER_LENGTH);
    if header[..2] != RESPONSE_SYNCHRONIZATION {
        return Err(invalid("FPGA response synchronization mismatch"));
    }
    if header[2] != sequence {
        return Err(invalid("FPGA response sequence mismatch"));
    }
    if header[3] == REJECTED_OPCODE || header[3] != command.opcode() {
        return Err(invalid("FPGA response opcode rejected or mismatched"));
    }
    let count = u32::from_le_bytes(header[12..16].try_into().expect("four count bytes"));
    if count as usize != node_count {
        return Err(invalid("FPGA response node count mismatch"));
    }
    let checksum = u16::from_le_bytes(checksum.try_into().expect("two checksum bytes"));
    if crc16(&checked[2..]) != checksum {
        return Err(invalid("FPGA response CRC16 mismatch"));
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
