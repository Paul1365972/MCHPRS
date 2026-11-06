mod expression;
mod inputs;
mod instance;
mod node_modules;

use crate::circuit::Circuit;
use crate::registers::{Contents, ReadbackLayout, Registers};
use expression::signal_name;
use include_dir::{include_dir, Dir};
use instance::{Instance, StatesSlice};
use mchprs_redpiler::Reporting;
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

pub(crate) fn write(
    registers: &Registers,
    reporting: Reporting,
    directory: &Path,
) -> io::Result<()> {
    let library = directory.join(LIBRARY_DIRECTORY);
    fs::create_dir_all(&library)?;
    LIBRARY.extract(library)?;
    fs::write(
        directory.join(CIRCUIT_FILE),
        Writer::new(registers, reporting).verilog(),
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
    reporting: Reporting,
    output: String,
}

impl<'a> Writer<'a> {
    fn new(registers: &'a Registers<'a>, reporting: Reporting) -> Self {
        Self {
            circuit: registers.circuit,
            registers,
            reporting,
            output: String::new(),
        }
    }

    fn line(&mut self, text: impl AsRef<str>) {
        self.output.push_str("    ");
        self.output.push_str(text.as_ref());
        self.output.push('\n');
    }

    fn verilog(mut self) -> String {
        let all: Vec<usize> = (0..self.circuit.nodes().len()).collect();
        let io_nodes = self.circuit.io_nodes();
        let reported = if self.reporting.reports_every_block_while_running() {
            &all
        } else {
            io_nodes
        };
        let layout = self.registers.readback_layout();
        let state_bits = layout.bytes(Contents::State, all.iter().copied()) * 8;
        let appearance_bits = layout.bytes(Contents::Appearance, reported.iter().copied()) * 8;
        let snapshot_bits = if self.reporting.snapshots_every_block() {
            state_bits
        } else {
            layout.bytes(Contents::State, io_nodes.iter().copied()) * 8
        };
        self.header(state_bits, appearance_bits, appearance_bits + snapshot_bits);
        let (instances, read) = self.instances(&layout);
        self.signal_declarations(&read);
        self.output.push('\n');
        for instance in &instances {
            self.line(instance.to_string());
        }
        self.output.push('\n');
        let used = layout.bits(Contents::State, all.iter().copied());
        if used != state_bits {
            self.line(format!("assign states[{}:{used}] = 0;", state_bits - 1));
        }
        let appearance = self.packed(&layout, Contents::Appearance, reported, appearance_bits);
        self.line(format!("assign appearance = {appearance};"));
        if self.reporting.snapshots_every_block() {
            self.line("assign readback = {appearance, states};");
        } else {
            let snapshot = self.packed(&layout, Contents::State, io_nodes, snapshot_bits);
            self.line(format!("assign readback = {{appearance, {snapshot}}};"));
        }
        self.output.push_str("endmodule\n");
        self.system(snapshot_bits / 8, appearance_bits / 8);
        self.output
    }

    fn header(&mut self, state_bits: usize, appearance_bits: usize, readback_bits: usize) {
        writeln!(
            self.output,
            "module compiled_circuit(
    input wire clock, set_input,
    input wire [31:0] input_index,
    input wire input_value, run,
    input wire [63:0] tick_limit,
    output wire busy, tick_started,
    output wire [63:0] tick_count,
    output wire [{}:0] readback
);
    wire [{}:0] states;
    wire [{}:0] appearance;
    function [3:0] maximum(input [3:0] left, input [3:0] right);
        maximum = left > right ? left : right;
    endfunction
    function [3:0] attenuated(input [3:0] signal, input [3:0] attenuation);
        attenuated = signal > attenuation ? signal - attenuation : 4'd0;
    endfunction
    wire input_nodes, begin_tick;
    assign tick_started = begin_tick;
    wire [3:0] firing;
    fpga_sequencer sequencer (
        .clock(clock), .set_input(set_input), .run(run), .tick_limit(tick_limit),
        .input_nodes(input_nodes), .begin_tick(begin_tick),
        .firing(firing), .busy(busy), .tick_count(tick_count)
    );
",
            readback_bits - 1,
            state_bits - 1,
            appearance_bits - 1
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

    fn packed(
        &self,
        layout: &ReadbackLayout,
        contents: Contents,
        nodes: &[usize],
        bits: usize,
    ) -> String {
        let mut slices: Vec<StatesSlice> = Vec::new();
        for &index in nodes {
            for (field, offset) in layout.node(index).read_fields(contents) {
                let width = field.width();
                match slices.last_mut() {
                    Some(last) if last.offset + last.width == offset => last.width += width,
                    _ => slices.push(StatesSlice { offset, width }),
                }
            }
        }
        let mut parts: Vec<_> = slices.iter().map(StatesSlice::to_string).collect();
        let padding = bits - layout.bits(contents, nodes.iter().copied());
        if padding != 0 {
            parts.push(format!("{padding}'d0"));
        }
        parts.reverse();
        concatenation(&parts)
    }

    fn system(&mut self, state_bytes: usize, appearance_bytes: usize) {
        let node_count = self.circuit.nodes().len();
        let identity = self.circuit.identity();
        writeln!(
            self.output,
            "
module circuit_system #(parameter CLOCKS_PER_BIT = 25)(
    input wire clock, serial_clock, serial_receive, clear_fault,
    output wire serial_transmit, ticking, halted,
    output wire [63:0] tick_count,
    output wire [3:0] fault_code
);
    wire set_input, input_value, run, tick_started;
    wire [31:0] input_index;
    wire [63:0] tick_limit;
    wire [{readback_high}:0] readback;
    protocol_controller #(
        .NODE_COUNT({node_count}), .STATE_BYTES({state_bytes}), .APPEARANCE_BYTES({appearance_bytes}),
        .CIRCUIT_ID(64'h{identity:016x}), .CLOCKS_PER_BIT(CLOCKS_PER_BIT)
    ) controller (
        .clock(clock), .serial_clock(serial_clock), .serial_receive(serial_receive),
        .clear_fault(clear_fault), .circuit_busy(ticking), .tick_started(tick_started), .tick_count(tick_count),
        .readback(readback), .serial_transmit(serial_transmit), .halted(halted),
        .fault_code(fault_code), .set_input(set_input), .input_index(input_index),
        .input_value(input_value), .run(run), .tick_limit(tick_limit)
    );
    compiled_circuit circuit (
        .clock(clock), .set_input(set_input), .input_index(input_index),
        .input_value(input_value), .run(run), .tick_limit(tick_limit), .busy(ticking),
        .tick_started(tick_started), .tick_count(tick_count), .readback(readback)
    );
endmodule",
            readback_high = (state_bytes + appearance_bytes) * 8 - 1,
        )
        .unwrap();
    }
}

fn concatenation(parts: &[String]) -> String {
    let rows: Vec<_> = parts
        .chunks(ITEMS_PER_LINE)
        .map(|row| row.join(", "))
        .collect();
    format!("{{\n        {}\n    }}", rows.join(",\n        "))
}
