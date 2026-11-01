mod expression;
mod inputs;
mod instance;
mod node_modules;

use self::expression::{signal_name, Moment};
use self::instance::Instance;
use self::node_modules::NodeModules;
use crate::circuit::Circuit;
use crate::registers::{readback_bytes, Registers};
use include_dir::{include_dir, Dir};
use std::collections::BTreeSet;
use std::fmt::Write;
use std::{fs, io, path::Path};

pub(crate) static RTL: Dir<'_> = include_dir!("$CARGO_MANIFEST_DIR/rtl");

const ITEMS_PER_LINE: usize = 16;

impl Circuit {
    pub(crate) fn write(&self, directory: &Path) -> io::Result<()> {
        let rtl = directory.join("rtl");
        fs::create_dir_all(&rtl)?;
        RTL.extract(rtl)?;
        fs::write(directory.join("circuit.v"), Writer::new(self).verilog())
    }
}

struct ReadbackLayout {
    offsets: Vec<usize>,
    widths: Vec<usize>,
}

impl ReadbackLayout {
    fn new(registers: &Registers) -> Self {
        let widths: Vec<usize> = (0..registers.circuit.nodes().len())
            .map(|index| {
                registers
                    .fields(index)
                    .iter()
                    .map(|field| field.width() as usize)
                    .sum()
            })
            .collect();
        let offsets = widths
            .iter()
            .scan(0, |offset, width| {
                let start = *offset;
                *offset += width;
                Some(start)
            })
            .collect();
        Self { offsets, widths }
    }

    fn bits(&self, nodes: &[usize]) -> usize {
        nodes.iter().map(|&index| self.widths[index]).sum()
    }

    fn padded_bits(&self, nodes: &[usize]) -> usize {
        readback_bytes(self.bits(nodes)) * 8
    }
}

struct Writer<'a> {
    circuit: &'a Circuit,
    registers: Registers<'a>,
    output: String,
}

impl<'a> Writer<'a> {
    fn new(circuit: &'a Circuit) -> Self {
        Self {
            circuit,
            registers: Registers::new(circuit),
            output: String::new(),
        }
    }

    fn line(&mut self, text: impl AsRef<str>) {
        self.output.push_str("    ");
        self.output.push_str(text.as_ref());
        self.output.push('\n');
    }

    fn verilog(mut self) -> String {
        let all: Vec<_> = (0..self.circuit.nodes().len()).collect();
        let io_nodes = self.circuit.io_nodes();
        let layout = ReadbackLayout::new(&self.registers);
        let (state_bits, output_bits) = (layout.padded_bits(&all), layout.padded_bits(&io_nodes));
        self.header(state_bits, output_bits);
        let (instances, read) = self.instances(&layout);
        self.signal_declarations(&read);
        self.output.push('\n');
        for instance in &instances {
            self.line(instance.to_string());
        }
        self.output.push('\n');
        let used = layout.bits(&all);
        if used != state_bits {
            self.line(format!("assign states[{}:{used}] = 0;", state_bits - 1));
        }
        let output_states = self.packed(&layout, &io_nodes, output_bits);
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

    fn instances(&self, layout: &ReadbackLayout) -> (Vec<Instance>, BTreeSet<(Moment, usize)>) {
        let builder = NodeModules::new(&self.registers);
        let mut instances: Vec<_> = (0..self.circuit.nodes().len())
            .filter_map(|index| builder.instance(index))
            .collect();
        let mut read = BTreeSet::new();
        for instance in &instances {
            instance.visit_signals(&mut |node, moment| {
                read.insert((moment, node));
            });
        }
        for instance in &mut instances {
            let index = instance.index();
            for moment in [Moment::BeforeFiring, Moment::AfterFiring] {
                if read.contains(&(moment, index)) {
                    instance.signal(moment);
                }
            }
            instance.readback(&self.registers.fields(index), layout.offsets[index]);
        }
        (instances, read)
    }

    fn signal_declarations(&mut self, read: &BTreeSet<(Moment, usize)>) {
        for moment in [Moment::BeforeFiring, Moment::AfterFiring] {
            let names: Vec<_> = read
                .iter()
                .filter(|(reader_moment, _)| *reader_moment == moment)
                .map(|&(moment, node)| signal_name(node, moment))
                .collect();
            for row in names.chunks(ITEMS_PER_LINE) {
                self.line(format!("wire [3:0] {};", row.join(", ")));
            }
        }
    }

    fn packed(&self, layout: &ReadbackLayout, nodes: &[usize], bits: usize) -> String {
        let mut parts: Vec<_> = nodes
            .iter()
            .filter(|&&index| layout.widths[index] != 0)
            .map(|&index| match layout.widths[index] {
                1 => format!("states[{}]", layout.offsets[index]),
                width => format!("states[{} +: {width}]", layout.offsets[index]),
            })
            .collect();
        let padding = bits - layout.bits(nodes);
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
        let identity = self.circuit.identity;
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
