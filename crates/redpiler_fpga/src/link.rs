use crate::protocol::{self, Contents, Hello, Mode, Readout, Request, FRAME_END};
use anyhow::{anyhow, Context};
use mchprs_redstone::input::Input;
use serialport::{ClearBuffer, SerialPort};
use std::fmt;
use std::io::{self, Read, Write};
use std::path::Path;
use std::time::Duration;

const ANSWER_TIMEOUT: Duration = Duration::from_millis(20);

pub trait ByteLink: fmt::Debug + Send {
    fn write(&mut self, bytes: &[u8]) -> io::Result<()>;

    /// Returns at least one byte. No byte within `timeout` is an `io::ErrorKind::TimedOut`.
    fn read(&mut self, buffer: &mut [u8], timeout: Duration) -> io::Result<usize>;
}

pub struct SerialLink(Box<dyn SerialPort>);

impl SerialLink {
    pub fn open(path: &Path, baud: u32) -> Result<SerialLink, serialport::Error> {
        let port = serialport::new(path.to_string_lossy(), baud).timeout(ANSWER_TIMEOUT);
        Ok(SerialLink(port.open()?))
    }

    pub fn discard_received(&mut self) -> Result<(), serialport::Error> {
        self.0.clear(ClearBuffer::Input)
    }
}

impl fmt::Debug for SerialLink {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_tuple("SerialLink")
            .field(&self.0.name())
            .finish()
    }
}

impl ByteLink for SerialLink {
    fn write(&mut self, bytes: &[u8]) -> io::Result<()> {
        self.0.write_all(bytes)
    }

    fn read(&mut self, buffer: &mut [u8], timeout: Duration) -> io::Result<usize> {
        self.0.set_timeout(timeout)?;
        match self.0.read(buffer)? {
            0 => Err(io::ErrorKind::UnexpectedEof.into()),
            count => Ok(count),
        }
    }
}

#[derive(Debug)]
pub struct Controller {
    link: Box<dyn ByteLink>,
    received: Vec<u8>,
}

impl Controller {
    pub fn new(link: Box<dyn ByteLink>) -> Controller {
        Controller {
            link,
            received: Vec::new(),
        }
    }

    pub fn hello(&mut self) -> anyhow::Result<Hello> {
        Ok(Hello::decode(self.request(Request::Hello)?))
    }

    pub fn set_mode(&mut self, mode: Mode) -> anyhow::Result<Readout> {
        self.readout(Request::Run(mode))
    }

    pub fn drive(&mut self, node: u32, input: Input) -> anyhow::Result<Readout> {
        self.readout(Request::Drive { node, input })
    }

    pub fn read(&mut self, contents: Contents) -> anyhow::Result<Readout> {
        self.readout(Request::Read(contents))
    }

    pub fn write_pll(&mut self, address: u8, data: u16) -> anyhow::Result<()> {
        self.command(Request::PllWrite { address, data })
    }

    pub fn read_pll(&mut self, address: u8) -> anyhow::Result<u16> {
        self.request(Request::PllRead { address })
            .map(u16::from_le_bytes)
    }

    pub fn release_clock(&mut self) -> anyhow::Result<()> {
        self.command(Request::ReleaseClock)
    }

    fn exchange(&mut self, request: Request) -> anyhow::Result<Vec<u8>> {
        self.link.write(&protocol::frame(&request.encode()))?;
        let end = loop {
            if let Some(end) = self.received.iter().position(|&byte| byte == FRAME_END) {
                break end;
            }
            let mut chunk = [0; 4096];
            let count = self.link.read(&mut chunk, ANSWER_TIMEOUT);
            let count = count.with_context(|| format!("the board did not answer {request:?}"))?;
            self.received.extend(&chunk[..count]);
        };
        let answer: Vec<u8> = self.received.drain(..=end).collect();
        Ok(protocol::payload(&answer[..end])?)
    }

    fn readout(&mut self, request: Request) -> anyhow::Result<Readout> {
        let payload = self.exchange(request)?;
        Readout::decode(&payload).with_context(|| {
            let length = payload.len();
            format!("the answer to {request:?} has only {length} bytes")
        })
    }

    fn command(&mut self, request: Request) -> anyhow::Result<()> {
        let [] = self.request(request)?;
        Ok(())
    }

    fn request<const N: usize>(&mut self, request: Request) -> anyhow::Result<[u8; N]> {
        let payload = self.exchange(request)?;
        let length = payload.len();
        payload
            .try_into()
            .map_err(|_| anyhow!("the answer to {request:?} has {length} bytes, not {N}"))
    }
}
