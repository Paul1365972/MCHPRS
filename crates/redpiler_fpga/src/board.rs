use crate::pll::PllLimits;
use crate::rtl::{self, source, CircuitSource, SourceFile, Sources};
use std::path::{Path, PathBuf};
use std::{fs, io};

const USB_SERIAL_PORTS: &str = "/sys/bus/usb-serial/devices";
const LATENCY_TIMER_MILLISECONDS: u32 = 1;

#[derive(Debug, Clone)]
pub struct Board {
    pub part: &'static str,
    pub chip_database_device: &'static str,
    pub prjxray_family: &'static str,
    pub slices: u32,
    pub oscillator_hertz: u32,
    pub pll: PllLimits,
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
    slices: 5_200,
    oscillator_hertz: 100_000_000,
    pll: PllLimits {
        vco_hertz: 800_000_000..=1_600_000_000,
        phase_detector_hertz: 19_000_000..=450_000_000,
        global_buffer_maximum_hertz: 464_000_000,
    },
    openfpgaloader_board: "basys3",
    usb_manufacturer: "Digilent",
    serial_interface_number: 1,
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
    #[error(
        "the latency timer of {} is {milliseconds} ms: write \
         {LATENCY_TIMER_MILLISECONDS} to {} as root",
        port.display(),
        timer.display()
    )]
    Latency {
        port: PathBuf,
        milliseconds: u32,
        timer: PathBuf,
    },
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
        let no_board = || {
            let no_board = BoardPortError::NoBoard(self.usb_manufacturer);
            given.clone().map_or(no_board, BoardPortError::NotUsb)
        };
        let Ok(entries) = fs::read_dir(USB_SERIAL_PORTS) else {
            return Err(no_board());
        };
        let ports = entries.flatten().filter_map(|entry| {
            let path = Path::new("/dev").join(entry.file_name());
            let interface = entry.path().join("..");
            let device = interface.join("..");
            let is_wanted = match &given {
                Some(given) => *given == path,
                None => {
                    attribute(&device, "manufacturer")? == self.usb_manufacturer
                        && attribute(&interface, "bInterfaceNumber")?
                            == format!("{:02x}", self.serial_interface_number)
                }
            };
            let usb_serial_number = attribute(&device, "serial")?;
            is_wanted.then_some((
                BoardPort {
                    path,
                    usb_serial_number,
                },
                entry.path().join("latency_timer"),
            ))
        });
        let mut ports: Vec<_> = ports.collect();
        let (port, timer) = match ports.len() {
            0 => return Err(no_board()),
            1 => ports.remove(0),
            _ => {
                let paths = ports.into_iter().map(|(port, _)| port.path).collect();
                return Err(BoardPortError::SeveralBoards(paths));
            }
        };
        let milliseconds = fs::read_to_string(&timer).ok();
        let milliseconds = milliseconds.and_then(|text| text.trim().parse::<u32>().ok());
        match milliseconds {
            Some(milliseconds) if milliseconds != LATENCY_TIMER_MILLISECONDS => {
                Err(BoardPortError::Latency {
                    port: port.path,
                    milliseconds,
                    timer,
                })
            }
            _ => Ok(port),
        }
    }

    fn package(&self) -> String {
        format!(
            "package board_pkg;\n    \
             localparam int OSCILLATOR_HERTZ = {};\n    \
             localparam int BAUD = {};\n\
             endpackage\n",
            self.oscillator_hertz, self.baud
        )
    }
}

fn attribute(directory: &Path, name: &str) -> Option<String> {
    let text = fs::read_to_string(directory.join(name)).ok()?;
    Some(text.trim().to_owned())
}

fn paths(ports: &[PathBuf]) -> String {
    let paths: Vec<_> = ports
        .iter()
        .map(|port| port.display().to_string())
        .collect();
    paths.join(", ")
}
