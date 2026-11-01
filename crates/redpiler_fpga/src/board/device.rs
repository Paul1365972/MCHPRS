use crate::circuit::Circuit;
use std::io;
use std::time::{Duration, Instant};

const TIMEOUT: Duration = Duration::from_secs(2);
const BAUD: u32 = 1_000_000;
const PROTOCOL_VERSION: u16 = 1;

#[derive(Clone, Copy, Debug)]
pub(super) enum Command {
    Snapshot,
    Input { index: u32, powered: bool },
    Identity,
    Status,
    RunUntil(u64),
    Outputs,
}

impl Command {
    fn encode(self) -> (u8, u64) {
        match self {
            Self::Snapshot => (0, 0),
            Self::Input { index, powered } => (2, u64::from(index) | (u64::from(powered) << 32)),
            Self::Identity => (4, 0),
            Self::Status => (7, 0),
            Self::RunUntil(tick) => (8, tick),
            Self::Outputs => (9, 0),
        }
    }
}

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
            node_count: circuit.nodes.len(),
            output_count: circuit.io_nodes().len(),
            state_bytes,
            output_bytes,
            sequence: 0,
        })
    }

    pub(super) fn execute(&mut self, command: Command) -> io::Result<Response> {
        let (opcode, argument) = command.encode();
        let port = self.port.as_mut().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::BrokenPipe,
                "FPGA device stopped after a previous failure",
            )
        })?;
        let frame = request_frame(self.sequence, opcode, argument);
        let (count, bytes) = if opcode == 9 {
            (self.output_count, self.output_bytes)
        } else {
            (self.node_count, self.state_bytes)
        };
        let mut response = vec![0; response_length(opcode, bytes)];
        let result = serial_exchange(port.as_mut(), &frame, &mut response)
            .and_then(|()| decode_response(&response, self.sequence, opcode, count));
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
    request: &[u8; 16],
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
                ))
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
                ))
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

fn request_frame(sequence: u8, opcode: u8, argument: u64) -> [u8; 16] {
    let mut frame = [0; 16];
    frame[..4].copy_from_slice(&[0xa5, 0x5a, sequence, opcode]);
    frame[4..12].copy_from_slice(&argument.to_le_bytes());
    frame[12..14].copy_from_slice(&PROTOCOL_VERSION.to_le_bytes());
    let checksum = crc16(&frame[2..14]);
    frame[14..].copy_from_slice(&checksum.to_le_bytes());
    frame
}

fn decode_response(
    response: &[u8],
    sequence: u8,
    opcode: u8,
    node_count: usize,
) -> io::Result<Response> {
    let invalid = |message| io::Error::new(io::ErrorKind::InvalidData, message);
    if response[..2] != [0x5a, 0xa5] {
        return Err(invalid("FPGA response synchronization mismatch"));
    }
    if response[2] != sequence {
        return Err(invalid("FPGA response sequence mismatch"));
    }
    if response[3] == 0xff || response[3] != opcode {
        return Err(invalid("FPGA response opcode rejected or mismatched"));
    }
    if u32::from_le_bytes(response[12..16].try_into().unwrap()) as usize != node_count {
        return Err(invalid("FPGA response node count mismatch"));
    }
    let checksum_offset = response.len() - 2;
    let checksum = u16::from_le_bytes([response[checksum_offset], response[checksum_offset + 1]]);
    if crc16(&response[2..checksum_offset]) != checksum {
        return Err(invalid("FPGA response CRC16 mismatch"));
    }
    Ok(Response {
        ticks: u64::from_le_bytes(response[4..12].try_into().unwrap()),
        data: response[16..checksum_offset].to_vec(),
    })
}

fn response_length(opcode: u8, readback_bytes: usize) -> usize {
    if (5..=8).contains(&opcode) {
        18
    } else {
        18 + readback_bytes
    }
}

fn exchange_timeout(response_length: usize) -> Duration {
    let wire_bits = (response_length as u64 + 16) * 10;
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
