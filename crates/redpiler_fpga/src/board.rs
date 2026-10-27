use crate::pll::Pll;
use crate::rtl::{self, source, CircuitSource, SourceFile, Sources};
use serialport::SerialPortType;
use std::path::{Path, PathBuf};
use std::{fs, io};

#[derive(Debug, Clone)]
pub struct Board {
    pub part: &'static str,
    pub chip_database_device: &'static str,
    pub prjxray_family: &'static str,
    pub slices: u32,
    pub oscillator_hertz: u32,
    pub pll: Pll,
    pub openfpgaloader_board: &'static str,
    pub usb_manufacturer: &'static str,
    pub serial_interface_number: u8,
    pub baud: u32,
    pub top_module: &'static str,
    pub top: SourceFile<'static>,
    pub constraints: SourceFile<'static>,
}

pub const BASYS3: Board = Board {
    part: "xc7a35tcpg236-1",
    // The XC7A35T has the die of the XC7A50T.
    chip_database_device: "xc7a50t",
    prjxray_family: "artix7",
    // The slices of the XC7A35T (DS180) and the oscillator of the Basys 3 (reference manual).
    slices: 5_200,
    oscillator_hertz: 100_000_000,
    pll: Pll {
        // Our choice: with the input dividers 3, 4 and 5 the VCO runs at 1,600, 1,200 and 960 MHz.
        multiplier: 48,
        // Speed grade -1 in DS181 v1.27.1: Table 38 for the PLL, Table 32 for the buffer.
        vco_hertz: 800_000_000..=1_600_000_000,
        phase_detector_hertz: 19_000_000..=450_000_000,
        global_buffer_maximum_hertz: 464_000_000,
    },
    openfpgaloader_board: "basys3",
    usb_manufacturer: "Digilent",
    // Of the FT2232HQ's two channels the first is JTAG and the second the UART.
    serial_interface_number: 1,
    // Our choice: 25 clocks of the oscillator per bit, and 12 MHz / 3 for the FT2232HQ.
    baud: 4_000_000,
    top_module: "basys3",
    top: source!("boards/basys3.sv"),
    constraints: source!("boards/basys3.xdc"),
};

const PACKAGE: &str = "board_pkg.sv";

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BoardPort {
    pub path: PathBuf,
    pub usb_serial_number: String,
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum BoardPortError {
    #[error("no serial port of a {0} board is attached")]
    NoBoard(&'static str),
    #[error("several boards are attached, choose one with --port: {}", paths(.0))]
    SeveralBoards(Vec<PathBuf>),
    #[error("{} is not the serial port of a USB device", .0.display())]
    NotUsb(PathBuf),
}

impl Board {
    pub fn write(&self, directory: &Path, circuit: &CircuitSource) -> io::Result<Sources> {
        let package = self.package();
        let package = SourceFile {
            path: PACKAGE,
            text: &package,
        };
        let top = [package].into_iter().chain(rtl::BOARD).chain([self.top]);
        Sources::write(directory, circuit, top)
    }

    pub fn port(&self, given: Option<&Path>) -> Result<BoardPort, BoardPortError> {
        let given = given.map(|given| fs::canonicalize(given).unwrap_or(given.to_owned()));
        let listed = serialport::available_ports().unwrap_or_default();
        let ports = listed.into_iter().filter_map(|port| {
            let SerialPortType::UsbPort(usb) = port.port_type else {
                return None;
            };
            let path = PathBuf::from(port.port_name);
            let is_wanted = match &given {
                Some(given) => *given == path,
                None => {
                    usb.manufacturer.as_deref() == Some(self.usb_manufacturer)
                        && usb.interface == Some(self.serial_interface_number)
                }
            };
            let usb_serial_number = usb.serial_number?;
            is_wanted.then_some(BoardPort {
                path,
                usb_serial_number,
            })
        });
        let mut ports: Vec<_> = ports.collect();
        match ports.len() {
            0 => {
                let no_board = BoardPortError::NoBoard(self.usb_manufacturer);
                Err(given.map_or(no_board, BoardPortError::NotUsb))
            }
            1 => Ok(ports.remove(0)),
            _ => {
                let paths = ports.into_iter().map(|port| port.path).collect();
                Err(BoardPortError::SeveralBoards(paths))
            }
        }
    }

    fn package(&self) -> String {
        let oscillator_hertz = self.oscillator_hertz;
        let setting = self
            .pll
            .fastest_setting(oscillator_hertz, oscillator_hertz.into());
        let setting = setting.expect("the PLL of a board reaches the frequency of its oscillator");
        format!(
            "package board_pkg;\n    \
             localparam int OSCILLATOR_HERTZ = {oscillator_hertz};\n    \
             localparam int BAUD = {};\n    \
             localparam int PLL_MULTIPLIER = {};\n    \
             localparam int PLL_INPUT_DIVIDER = {};\n    \
             localparam int PLL_OUTPUT_DIVIDER = {};\n\
             endpackage\n",
            self.baud, self.pll.multiplier, setting.input_divider, setting.output_divider
        )
    }
}

fn paths(ports: &[PathBuf]) -> String {
    let paths: Vec<_> = ports
        .iter()
        .map(|port| port.display().to_string())
        .collect();
    paths.join(", ")
}
