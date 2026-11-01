mod expression;
mod inputs;
mod instance;
mod node_modules;

use crate::circuit::Circuit;
use crate::registers::{ReadbackLayout, Registers};
use expression::signal_name;
use include_dir::{include_dir, Dir};
use instance::{Instance, StatesSlice};
use node_modules::NodeModules;
use std::collections::BTreeSet;
use std::fmt::Write;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

static LIBRARY: Dir<'_> = include_dir!("$CARGO_MANIFEST_DIR/rtl");

pub(crate) const LIBRARY_DIRECTORY: &str = "rtl";
const CIRCUIT_FILE: &str = "circuit.v";
const ITEMS_PER_LINE: usize = 16;

pub(crate) fn write(registers: &Registers, directory: &Path) -> io::Result<()> {
    let library = directory.join(LIBRARY_DIRECTORY);
    fs::create_dir_all(&library)?;
    LIBRARY.extract(library)?;
    fs::write(
        directory.join(CIRCUIT_FILE),
        Writer::new(registers).verilog(),
    )
}

/// The Verilog sources that `write` creates, relative to its directory, with the circuit last.
pub(crate) fn sources() -> Vec<PathBuf> {
    let mut sources: Vec<_> = LIBRARY
        .find("**/*.v")
        .expect("valid Verilog glob")
        .map(|entry| Path::new(LIBRARY_DIRECTORY).join(entry.path()))
        .collect();
    sources.sort();
    sources.push(PathBuf::from(CIRCUIT_FILE));
    sources
}

struct Writer<'a> {
    circuit: &'a Circuit,
    registers: &'a Registers<'a>,
    output: String,
}

impl<'a> Writer<'a> {
    fn new(registers: &'a Registers<'a>) -> Self {
        Self {
            circuit: registers.circuit,
            registers,
            output: String::new(),
        }
    }

    fn line(&mut self, text: impl AsRef<str>) {
        self.output.push_str("    ");
        self.output.push_str(text.as_ref());
        self.output.push('\n');
    }

    fn verilog(mut self) -> String {
        let all = 0..self.circuit.nodes().len();
        let io_nodes = self.circuit.io_nodes();
        let layout = self.registers.readback_layout();
        let state_bits = layout.bytes(all.clone()) * 8;
        let output_bits = layout.bytes(io_nodes.iter().copied()) * 8;
        self.header(state_bits, output_bits);
        let (instances, read) = self.instances(&layout);
        self.signal_declarations(&read);
        self.output.push('\n');
        for instance in &instances {
            self.line(instance.to_string());
        }
        self.output.push('\n');
        let used = layout.bits(all);
        if used != state_bits {
            self.line(format!("assign states[{}:{used}] = 0;", state_bits - 1));
        }
        let output_states = self.packed(&layout, io_nodes, output_bits);
        self.line(format!("assign output_states = {output_states};"));
        self.output.push_str("endmodule\n");
        self.system(state_bits / 8, output_bits / 8);
        self.output
    }

    fn header(&mut self, state_bits: usize, output_bits: usize) {
        writeln!(
            self.output,
            "module compiled_circuit(
    input wire clock, set_input,
    input wire [31:0] input_index,
    input wire input_value, run,
    input wire [63:0] tick_limit,
    output wire busy,
    output wire [63:0] tick_count,
    output wire [{}:0] states,
    output wire [{}:0] output_states
);
    function [3:0] maximum(input [3:0] left, input [3:0] right);
        maximum = left > right ? left : right;
    endfunction
    function [3:0] attenuated(input [3:0] signal, input [3:0] attenuation);
        attenuated = signal > attenuation ? signal - attenuation : 4'd0;
    endfunction
    wire input_nodes, begin_tick;
    wire [3:0] firing;
    fpga_sequencer sequencer (
        .clock(clock), .set_input(set_input), .run(run), .tick_limit(tick_limit),
        .input_nodes(input_nodes), .begin_tick(begin_tick),
        .firing(firing), .busy(busy), .tick_count(tick_count)
    );
",
            state_bits - 1,
            output_bits - 1
        )
        .unwrap();
    }

    fn instances(&self, layout: &ReadbackLayout) -> (Vec<Instance>, BTreeSet<usize>) {
        let node_modules = NodeModules::new(self.registers);
        let mut instances: Vec<_> = (0..self.circuit.nodes().len())
            .filter_map(|index| node_modules.instance(index))
            .collect();
        let mut read = BTreeSet::new();
        for instance in &instances {
            instance.visit_signals(&mut |node| {
                read.insert(node);
            });
        }
        for instance in &mut instances {
            let index = instance.index();
            if read.contains(&index) {
                instance.signal();
            }
            instance.readback(layout.node(index));
        }
        (instances, read)
    }

    fn signal_declarations(&mut self, read: &BTreeSet<usize>) {
        let names: Vec<_> = read.iter().map(|&node| signal_name(node)).collect();
        for row in names.chunks(ITEMS_PER_LINE) {
            self.line(format!("wire [3:0] {};", row.join(", ")));
        }
    }

    fn packed(&self, layout: &ReadbackLayout, nodes: &[usize], bits: usize) -> String {
        let mut parts: Vec<_> = nodes
            .iter()
            .map(|&index| layout.node(index))
            .filter(|node| node.width != 0)
            .map(|node| {
                StatesSlice {
                    offset: node.offset,
                    width: node.width,
                }
                .to_string()
            })
            .collect();
        let padding = bits - layout.bits(nodes.iter().copied());
        if padding != 0 {
            parts.push(format!("{padding}'d0"));
        }
        parts.reverse();
        let rows: Vec<_> = parts
            .chunks(ITEMS_PER_LINE)
            .map(|row| row.join(", "))
            .collect();
        format!("{{\n        {}\n    }}", rows.join(",\n        "))
    }

    fn system(&mut self, state_bytes: usize, output_bytes: usize) {
        let node_count = self.circuit.nodes().len();
        let output_count = self.circuit.io_nodes().len();
        let identity = self.circuit.identity();
        writeln!(
            self.output,
            "
module circuit_system #(parameter CLOCKS_PER_BIT = 100)(
    input wire clock, serial_receive,
    output wire serial_transmit, busy,
    output wire [63:0] tick_count,
    output wire fault
);
    protocol_controller #(
        .NODE_COUNT({node_count}), .OUTPUT_COUNT({output_count}),
        .STATE_BYTES({state_bytes}), .OUTPUT_BYTES({output_bytes}),
        .CIRCUIT_ID(64'h{identity:016x}), .CLOCKS_PER_BIT(CLOCKS_PER_BIT)
    ) controller (
        .clock(clock), .serial_receive(serial_receive), .serial_transmit(serial_transmit),
        .busy(busy), .tick_count(tick_count), .fault(fault)
    );
endmodule"
        )
        .unwrap();
    }
}
