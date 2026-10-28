use crate::protocol::{self, Contents, Hello, Mode, Readout, Request, FRAME_END};
use anyhow::{anyhow, Context};
use mchprs_redstone::input::Input;
use std::fmt;
use std::io;

pub trait ByteLink: fmt::Debug + Send {
    fn write(&mut self, bytes: &[u8]) -> io::Result<()>;

    /// Returns at least one byte, or an error.
    fn read(&mut self, buffer: &mut [u8]) -> io::Result<usize>;
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

    fn request<const N: usize>(&mut self, request: Request) -> anyhow::Result<[u8; N]> {
        let payload = self.exchange(request)?;
        let length = payload.len();
        payload
            .try_into()
            .map_err(|_| anyhow!("the answer to {request:?} has {length} bytes, not {N}"))
    }
}
