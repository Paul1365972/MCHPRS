use crate::protocol::{self, Contents, Hello, Mode, Readout, Request, FRAME_END};
use anyhow::{anyhow, Context};
use mchprs_redstone::input::Input;
use serialport::{ClearBuffer, SerialPort};
use std::fmt;
use std::io::{self, Read, Write};
use std::path::Path;
use std::time::Duration;

const ANSWER_TIMEOUT: Duration = Duration::from_secs(1);

pub trait ByteLink: fmt::Debug + Send {
    fn write(&mut self, bytes: &[u8]) -> io::Result<()>;

    /// Returns at least one byte, or an error.
    fn read(&mut self, buffer: &mut [u8]) -> io::Result<usize>;
}

pub struct SerialLink(Box<dyn SerialPort>);

impl SerialLink {
    pub fn open(path: &Path, baud: u32) -> Result<SerialLink, serialport::Error> {
        let port = serialport::new(path.to_string_lossy(), baud).timeout(ANSWER_TIMEOUT);
        let port = port.open_native()?;
        #[cfg(target_os = "linux")]
        if let Err(error) = request_low_latency(&port) {
            tracing::warn!("{} keeps its latency: {error}", path.display());
        }
        Ok(SerialLink(Box::new(port)))
    }

    pub fn discard_received(&mut self) -> Result<(), serialport::Error> {
        self.0.clear(ClearBuffer::Input)
    }
}

/// An FTDI adapter holds back what it receives for up to 16 ms unless the port asks for low
/// latency.
#[cfg(target_os = "linux")]
fn request_low_latency(port: &serialport::TTYPort) -> io::Result<()> {
    use nix::libc::{c_int, ioctl, TIOCGSERIAL, TIOCSSERIAL};
    use std::os::fd::AsRawFd;

    // As in Linux's `include/uapi/linux/tty_flags.h`; the struct is from `serial.h` beside it.
    const ASYNC_LOW_LATENCY: c_int = 1 << 13;
    /// Linux's `struct serial_struct` up to its flags, with room for the rest.
    #[repr(C)]
    struct SerialStruct {
        before_flags: [c_int; 4],
        flags: c_int,
        after_flags: [u64; 8],
    }

    let mut serial = SerialStruct {
        before_flags: [0; 4],
        flags: 0,
        after_flags: [0; 8],
    };
    let port = port.as_raw_fd();
    // SAFETY: the kernel writes at most a `struct serial_struct`, which fits in `serial`.
    if unsafe { ioctl(port, TIOCGSERIAL, &raw mut serial) } != 0 {
        return Err(io::Error::last_os_error());
    }
    serial.flags |= ASYNC_LOW_LATENCY;
    // SAFETY: the kernel reads at most a `struct serial_struct`, which fits in `serial`.
    if unsafe { ioctl(port, TIOCSSERIAL, &raw const serial) } != 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(())
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

    fn read(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
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
            let count = self.link.read(&mut chunk);
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
