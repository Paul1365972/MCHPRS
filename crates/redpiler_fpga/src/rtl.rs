use super::circuit::{Circuit, ComparatorMode, Kind, Link};
use super::reference::is_initially_settled;
use super::registers::{readback_bytes, timer, Lane, Registers};
use include_dir::{include_dir, Dir};
use std::fmt::Write;
use std::{fs, io, path::Path};

pub(crate) static RTL: Dir<'_> = include_dir!("$CARGO_MANIFEST_DIR/rtl");

#[derive(Clone, Copy, PartialEq, Eq)]
enum Ports {
    Board,
    Simulation,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Value {
    Current,
    Next,
}

impl Value {
    fn prefix(self) -> &'static str {
        match self {
            Value::Current => "",
            Value::Next => "next_",
        }
    }
}

impl Circuit {
    pub(crate) fn write(&self, directory: &Path) -> io::Result<()> {
        self.write_circuit(directory, Ports::Board)
    }

    pub(crate) fn write_simulation(&self, directory: &Path) -> io::Result<()> {
        self.write_circuit(directory, Ports::Simulation)
    }

    fn write_circuit(&self, directory: &Path, ports: Ports) -> io::Result<()> {
        let rtl = directory.join("rtl");
        fs::create_dir_all(&rtl)?;
        RTL.extract(rtl)?;
        let mut emitter = Emitter {
            circuit: self,
            registers: Registers::new(self),
            output: String::new(),
        };
        emitter.circuit_module(ports);
        if ports == Ports::Board {
            emitter.system_module();
        }
        fs::write(directory.join("circuit.v"), emitter.output)
    }
}

fn countdown(delay: u8) -> u32 {
    match delay {
        0 => 0,
        delay => 1 << (delay - 1),
    }
}

fn or_all(terms: Vec<String>) -> String {
    match terms.len() {
        0 => "1'b0".to_owned(),
        _ if terms.iter().any(|term| term == "1'b1") => "1'b1".to_owned(),
        _ => terms.join(" || "),
    }
}

fn and_all(terms: Vec<String>) -> String {
    let terms: Vec<_> = terms.into_iter().filter(|term| term != "1'b1").collect();
    match terms.len() {
        0 => "1'b1".to_owned(),
        _ if terms.iter().any(|term| term == "1'b0") => "1'b0".to_owned(),
        _ => terms.join(" && "),
    }
}

fn maximum_tree(terms: &[String]) -> String {
    if terms.len() == 1 {
        return terms[0].clone();
    }
    let (left, right) = terms.split_at(terms.len() / 2);
    format!("maximum({}, {})", maximum_tree(left), maximum_tree(right))
}

fn bit(value: bool) -> String {
    format!("1'b{}", u8::from(value))
}

fn range(width: u32) -> String {
    if width == 1 {
        String::new()
    } else {
        format!("[{}:0] ", width - 1)
    }
}

/// The Verilog of one node: registers and the post-fire values other nodes read are declared
/// before any logic, so a node's logic can read every other node regardless of order.
#[derive(Default)]
struct NodeVerilog {
    registers: Vec<String>,
    outputs: Vec<String>,
    logic: Vec<String>,
    updates: Vec<String>,
}

impl NodeVerilog {
    fn register(&mut self, width: u32, name: String, initial: String) {
        self.registers
            .push(format!("reg {}{name} = {initial};", range(width)));
    }

    fn vector_register(&mut self, width: u32, name: String, initial: String) {
        self.registers
            .push(format!("reg [{}:0] {name} = {initial};", width - 1));
    }

    fn output(&mut self, width: u32, name: String, expression: String) {
        self.outputs.push(format!("wire {}{name};", range(width)));
        self.logic.push(format!("assign {name} = {expression};"));
    }

    fn wire(&mut self, width: u32, name: String, expression: String) {
        self.logic
            .push(format!("wire {}{name} = {expression};", range(width)));
    }

    fn update(&mut self, statement: String) {
        self.updates.push(statement);
    }
}

struct Emitter<'a> {
    circuit: &'a Circuit,
    registers: Registers<'a>,
    output: String,
}

impl<'a> Emitter<'a> {
    fn line(&mut self, text: impl AsRef<str>) {
        self.output.push_str("    ");
        self.output.push_str(text.as_ref());
        self.output.push('\n');
    }

    fn reachable(&self, link: &Link) -> u16 {
        let strengths = self.registers.strengths(link.source);
        (0_u8..=15)
            .filter(|strength| strengths & (1 << strength) != 0)
            .fold(0, |reachable, strength| {
                reachable | (1 << strength.saturating_sub(link.attenuation))
            })
    }

    fn strength(&self, source: usize, value: Value) -> String {
        if let Some(strength) = self.registers.literal(source) {
            return format!("4'd{strength}");
        }
        let node = &self.circuit.nodes[source];
        let prefix = value.prefix();
        match node.kind {
            kind if kind.has_boolean_output() => {
                let powered = format!("{{4{{{prefix}powered_{source}}}}}");
                if self.registers.has_imported_flag(source) {
                    let strength = node.initial.strength;
                    format!("({prefix}imported_{source} ? 4'd{strength} : {powered})")
                } else {
                    powered
                }
            }
            Kind::Comparator { .. } => match self.registers.two_level(source) {
                Some((low, high)) => format!("({prefix}high_{source} ? 4'd{high} : 4'd{low})"),
                None => format!("{prefix}strength_{source}"),
            },
            Kind::Lamp => format!(
                "({prefix}cleared_{source} ? 4'd0 : 4'd{})",
                node.initial.strength
            ),
            kind => unreachable!("{kind:?} has a constant output strength"),
        }
    }

    fn nonzero(&self, source: usize, value: Value) -> String {
        let kind = self.circuit.nodes[source].kind;
        let prefix = value.prefix();
        if kind.has_boolean_output() && !self.registers.imports_strength(source) {
            format!("{prefix}powered_{source}")
        } else if self
            .registers
            .two_level(source)
            .is_some_and(|(low, _)| low == 0)
        {
            format!("{prefix}high_{source}")
        } else {
            format!("{} != 4'd0", self.strength(source, value))
        }
    }

    fn attenuated(&self, link: &Link, value: Value) -> String {
        let signal = self.strength(link.source, value);
        if link.attenuation == 0 {
            signal
        } else {
            let attenuation = link.attenuation;
            format!("({signal} > 4'd{attenuation} ? {signal} - 4'd{attenuation} : 4'd0)")
        }
    }

    fn powers(&self, link: &Link, value: Value) -> Option<String> {
        let reachable = self.reachable(link);
        if reachable == 1 {
            return None;
        }
        if reachable & 1 == 0 {
            return Some("1'b1".to_owned());
        }
        let strengths = self.registers.strengths(link.source);
        let attenuation = link.attenuation;
        if (1..=attenuation).all(|strength| strengths & (1 << strength) == 0) {
            Some(self.nonzero(link.source, value))
        } else {
            Some(format!(
                "{} > 4'd{attenuation}",
                self.strength(link.source, value)
            ))
        }
    }

    fn boolean_input(&self, links: &[Link], value: Value) -> String {
        or_all(
            links
                .iter()
                .filter_map(|link| self.powers(link, value))
                .collect(),
        )
    }

    fn strength_input(&self, links: &[Link], value: Value) -> String {
        let values = links
            .iter()
            .fold(1, |values, link| values | self.reachable(link));
        if values == 1 {
            return "4'd0".to_owned();
        }
        if values.count_ones() == 2 {
            let high = 15 - values.leading_zeros();
            return match self.boolean_input(links, value).as_str() {
                "1'b1" => format!("4'd{high}"),
                powered => format!("({powered}) ? 4'd{high} : 4'd0"),
            };
        }
        let mut constant = 0;
        let mut terms = Vec::new();
        for link in links.iter().filter(|link| self.reachable(link) != 1) {
            match self.registers.literal(link.source) {
                Some(strength) => constant = constant.max(strength - link.attenuation),
                None => terms.push(self.attenuated(link, value)),
            }
        }
        if constant != 0 {
            terms.push(format!("4'd{constant}"));
        }
        maximum_tree(&terms)
    }

    fn changed(&self, link: &Link) -> Option<String> {
        if self.reachable(link).count_ones() < 2 {
            return None;
        }
        let source = link.source;
        let strengths = self.registers.strengths(source);
        Some(if strengths.count_ones() == 2 && strengths & 1 != 0 {
            format!(
                "({}) != ({})",
                self.nonzero(source, Value::Next),
                self.nonzero(source, Value::Current)
            )
        } else {
            format!(
                "{} != {}",
                self.attenuated(link, Value::Next),
                self.attenuated(link, Value::Current)
            )
        })
    }

    fn initial_strength_input(&self, links: &[Link]) -> u8 {
        links
            .iter()
            .map(|link| {
                self.circuit.nodes[link.source]
                    .initial
                    .strength
                    .saturating_sub(link.attenuation)
            })
            .max()
            .unwrap_or(0)
    }

    fn initial_desired(&self, index: usize) -> u8 {
        let node = &self.circuit.nodes[index];
        let Kind::Comparator {
            mode, far_input, ..
        } = node.kind
        else {
            unreachable!("only comparators have a desired strength");
        };
        let main = self.initial_strength_input(&node.inputs);
        let rear = match far_input {
            Some(far) if main < 15 => far,
            _ => main,
        };
        let side = self.initial_strength_input(&node.sides);
        if rear < side {
            0
        } else if mode == ComparatorMode::Subtract {
            rear - side
        } else {
            rear
        }
    }

    fn circuit_module(&mut self, ports: Ports) {
        let count = self.circuit.nodes.len();
        let all: Vec<_> = (0..count).collect();
        let mut state_ports = format!("[{}:0] states", self.readback_bits(&all) - 1);
        if ports == Ports::Board {
            let io_nodes = self.circuit.io_nodes();
            state_ports.push_str(&format!(
                ",\n    output wire [{}:0] output_states",
                self.readback_bits(&io_nodes) - 1
            ));
        }
        writeln!(
            self.output,
            "module compiled_circuit(
    input wire clock, set_input,
    input wire [31:0] input_index,
    input wire input_value, run,
    input wire [63:0] tick_limit,
    output wire busy,
    output wire [63:0] tick_count,
    output wire {state_ports}
);
    function [3:0] maximum(input [3:0] left, input [3:0] right);
        maximum = left > right ? left : right;
    endfunction
    wire input_nodes, begin_tick;
    wire [3:0] firing;
    fpga_sequencer sequencer (
        .clock(clock), .set_input(set_input), .run(run), .tick_limit(tick_limit),
        .input_nodes(input_nodes), .begin_tick(begin_tick),
        .firing(firing), .busy(busy), .tick_count(tick_count)
    );"
        )
        .unwrap();
        let nodes: Vec<_> = (0..count).map(|index| self.node(index)).collect();
        self.output.push('\n');
        self.line("// State of every node, and the post-fire values other nodes read");
        for node in &nodes {
            let declarations: Vec<_> = node
                .registers
                .iter()
                .chain(&node.outputs)
                .cloned()
                .collect();
            if !declarations.is_empty() {
                self.line(declarations.join(" "));
            }
        }
        for (index, node) in nodes.iter().enumerate() {
            if node.logic.is_empty() && node.updates.is_empty() {
                continue;
            }
            self.output.push('\n');
            let description = self.description(index);
            self.line(format!("// Node {index}: {description}"));
            for line in &node.logic {
                self.line(line);
            }
            if !node.updates.is_empty() {
                self.line("always @(posedge clock) begin");
                for update in &node.updates {
                    self.line(format!("    {update}"));
                }
                self.line("end");
            }
        }
        self.output.push('\n');
        let states = self.packed(&all);
        self.line(format!("assign states = {states};"));
        if ports == Ports::Board {
            let output_states = self.packed(&self.circuit.io_nodes());
            self.line(format!("assign output_states = {output_states};"));
        }
        self.output.push_str("endmodule\n");
    }

    fn system_module(&mut self) {
        let count = self.circuit.nodes.len();
        let io_nodes = self.circuit.io_nodes();
        let output_count = io_nodes.len();
        let all: Vec<_> = (0..count).collect();
        let state_bytes = self.readback_bits(&all) / 8;
        let output_bytes = self.readback_bits(&io_nodes) / 8;
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
        .NODE_COUNT({count}), .OUTPUT_COUNT({output_count}), .STATE_BYTES({state_bytes}), .OUTPUT_BYTES({output_bytes}),
        .CIRCUIT_ID(64'h{:016x}), .CLOCKS_PER_BIT(CLOCKS_PER_BIT)
    ) controller (
        .clock(clock), .serial_receive(serial_receive), .serial_transmit(serial_transmit),
        .busy(busy), .tick_count(tick_count), .fault(fault)
    );
endmodule",
            self.circuit.identity
        )
        .unwrap();
    }

    fn description(&self, index: usize) -> String {
        match self.circuit.nodes[index].kind {
            Kind::Lever => "lever".to_owned(),
            Kind::Button => "button".to_owned(),
            Kind::PressurePlate => "pressure plate".to_owned(),
            Kind::Torch => "torch".to_owned(),
            Kind::Repeater {
                delay,
                facing_diode,
            } => {
                let facing = if facing_diode { ", facing a diode" } else { "" };
                format!("repeater, delay {delay}{facing}")
            }
            Kind::Comparator {
                mode,
                far_input,
                facing_diode,
            } => {
                let mode = match mode {
                    ComparatorMode::Compare => "compare",
                    ComparatorMode::Subtract => "subtract",
                };
                let far = far_input.map_or(String::new(), |far| format!(", far input {far}"));
                let facing = if facing_diode { ", facing a diode" } else { "" };
                format!("comparator, {mode}{far}{facing}")
            }
            Kind::Lamp => "lamp".to_owned(),
            Kind::Trapdoor => "trapdoor".to_owned(),
            Kind::Wire => "wire".to_owned(),
            Kind::Constant => "constant".to_owned(),
        }
    }

    fn has_side_strength(&self, index: usize) -> bool {
        let sides = &self.circuit.nodes[index].sides;
        sides.iter().any(|link| self.reachable(link) != 1)
    }

    fn node(&self, index: usize) -> NodeVerilog {
        let mut node = NodeVerilog::default();
        if timer(self.circuit.nodes[index].kind).is_some() {
            self.timer(index, &mut node);
        }
        match self.circuit.nodes[index].kind {
            Kind::Constant => {}
            Kind::Lever | Kind::PressurePlate => self.lever(index, &mut node),
            Kind::Button => self.button(index, &mut node),
            Kind::Torch => self.torch(index, &mut node),
            Kind::Repeater { .. } => self.repeater(index, &mut node),
            Kind::Comparator { .. } => self.comparator(index, &mut node),
            Kind::Lamp => self.lamp(index, &mut node),
            Kind::Trapdoor => self.trapdoor(index, &mut node),
            Kind::Wire => self.wire(index, &mut node),
        }
        node
    }

    fn selected(&self, index: usize) -> String {
        let width = (usize::BITS - (self.circuit.nodes.len() - 1).leading_zeros()).max(1);
        format!(
            "input_nodes && input_index[{}:0] == {width}'d{index}",
            width - 1
        )
    }

    /// Settles every clock, except a node imported unsettled, which waits for an input change.
    fn update(&self, index: usize, node: &mut NodeVerilog) -> String {
        if is_initially_settled(self.circuit, index) {
            return "1'b1".to_owned();
        }
        let definition = &self.circuit.nodes[index];
        let mut changes: Vec<_> = definition
            .inputs
            .iter()
            .chain(&definition.sides)
            .filter_map(|link| self.changed(link))
            .map(|change| format!("({change})"))
            .collect();
        changes.sort();
        changes.dedup();
        node.register(1, format!("stale_{index}"), bit(true));
        node.wire(1, format!("changed_{index}"), or_all(changes));
        node.wire(
            1,
            format!("update_{index}"),
            format!("!stale_{index} || changed_{index}"),
        );
        node.update(format!(
            "stale_{index} <= stale_{index} && !changed_{index};"
        ));
        format!("update_{index}")
    }

    fn boolean_input_wire(&self, index: usize, node: &mut NodeVerilog) {
        let input = self.boolean_input(&self.circuit.nodes[index].inputs, Value::Next);
        node.wire(1, format!("input_{index}"), input);
    }

    fn previous_input(&self, index: usize, node: &mut NodeVerilog, input: &str) {
        let initial = bit(self.initial_strength_input(&self.circuit.nodes[index].inputs) != 0);
        node.register(1, format!("previous_input_{index}"), initial);
        node.update(format!("previous_input_{index} <= {input};"));
    }

    fn boolean_output(&self, index: usize, node: &mut NodeVerilog, write: String, next: String) {
        let initial = self.circuit.nodes[index].initial;
        node.register(1, format!("powered_{index}"), bit(initial.powered));
        node.wire(1, format!("write_{index}"), write);
        node.output(
            1,
            format!("next_powered_{index}"),
            format!("write_{index} ? {next} : powered_{index}"),
        );
        node.update(format!("powered_{index} <= next_powered_{index};"));
        if self.registers.has_imported_flag(index) {
            node.register(1, format!("imported_{index}"), bit(true));
            node.output(
                1,
                format!("next_imported_{index}"),
                format!("imported_{index} && !write_{index}"),
            );
            node.update(format!("imported_{index} <= next_imported_{index};"));
        }
    }

    fn lever(&self, index: usize, node: &mut NodeVerilog) {
        let selected = self.selected(index);
        self.boolean_output(index, node, selected, "input_value".to_owned());
    }

    fn button(&self, index: usize, node: &mut NodeVerilog) {
        let selected = self.selected(index);
        node.wire(
            1,
            format!("pressed_{index}"),
            format!("{selected} && input_value && !powered_{index}"),
        );
        self.boolean_output(
            index,
            node,
            format!("pressed_{index} || fire_{index} && powered_{index}"),
            format!("pressed_{index}"),
        );
        self.schedule(index, node, None, Some((format!("pressed_{index}"), 3)));
    }

    fn torch(&self, index: usize, node: &mut NodeVerilog) {
        self.boolean_output(
            index,
            node,
            format!("fire_{index} && powered_{index} == previous_input_{index}"),
            format!("!previous_input_{index}"),
        );
        let update = self.update(index, node);
        self.boolean_input_wire(index, node);
        self.previous_input(index, node, &format!("input_{index}"));
        self.pending(index, node, None);
        let schedule = and_all(vec![
            update,
            format!("!pending_{index}"),
            format!("next_powered_{index} == input_{index}"),
        ]);
        self.schedule(index, node, Some(schedule), None);
    }

    fn repeater(&self, index: usize, node: &mut NodeVerilog) {
        let definition = &self.circuit.nodes[index];
        let unlocked = if self.registers.has_lock(index) {
            node.register(1, format!("locked_{index}"), bit(definition.initial.locked));
            format!("!locked_{index}")
        } else {
            "1'b1".to_owned()
        };
        node.wire(
            1,
            format!("pulse_{index}"),
            and_all(vec![
                format!("fire_{index}"),
                unlocked.clone(),
                format!("!powered_{index}"),
                format!("!previous_input_{index}"),
            ]),
        );
        self.boolean_output(
            index,
            node,
            and_all(vec![
                format!("fire_{index}"),
                unlocked,
                format!("(!powered_{index} || !previous_input_{index})"),
            ]),
            format!("!powered_{index}"),
        );
        let update = self.update(index, node);
        self.boolean_input_wire(index, node);
        self.previous_input(index, node, &format!("input_{index}"));
        let unlocked_now = if definition.sides.is_empty() {
            "1'b1".to_owned()
        } else {
            let side = self.boolean_input(&definition.sides, Value::Next);
            node.wire(1, format!("side_{index}"), side);
            format!("!side_{index}")
        };
        if self.registers.has_lock(index) {
            let side = if definition.sides.is_empty() {
                bit(false)
            } else {
                format!("side_{index}")
            };
            node.update(if update == "1'b1" {
                format!("locked_{index} <= {side};")
            } else {
                format!("if ({update}) locked_{index} <= {side};")
            });
        }
        self.pending(index, node, Some(format!("pulse_{index}")));
        let schedule = and_all(vec![
            update,
            unlocked_now,
            format!("!pending_{index}"),
            format!("next_powered_{index} != input_{index}"),
        ]);
        self.schedule(
            index,
            node,
            Some(schedule),
            Some((format!("pulse_{index}"), 1)),
        );
    }

    fn comparator(&self, index: usize, node: &mut NodeVerilog) {
        let definition = &self.circuit.nodes[index];
        let Kind::Comparator {
            mode, far_input, ..
        } = definition.kind
        else {
            unreachable!("comparator");
        };
        let initial = definition.initial;
        let (output, previous_width, next_strength) = match self.registers.two_level(index) {
            Some((low, high)) => {
                node.register(
                    1,
                    format!("previous_input_{index}"),
                    bit(self.initial_desired(index) == high),
                );
                node.register(1, format!("high_{index}"), bit(initial.strength == high));
                (
                    format!("high_{index}"),
                    1,
                    format!("(next_high_{index} ? 4'd{high} : 4'd{low})"),
                )
            }
            None => {
                node.register(
                    4,
                    format!("previous_input_{index}"),
                    format!("4'd{}", self.initial_desired(index)),
                );
                node.register(
                    4,
                    format!("strength_{index}"),
                    format!("4'd{}", initial.strength),
                );
                (
                    format!("strength_{index}"),
                    4,
                    format!("next_strength_{index}"),
                )
            }
        };
        node.wire(
            1,
            format!("write_{index}"),
            format!("fire_{index} && previous_input_{index} != {output}"),
        );
        node.output(
            previous_width,
            format!("next_{output}"),
            format!("write_{index} ? previous_input_{index} : {output}"),
        );
        node.update(format!("{output} <= next_{output};"));
        if self.registers.has_imported_flag(index) {
            node.register(1, format!("imported_{index}"), bit(true));
            node.output(
                1,
                format!("next_imported_{index}"),
                format!("imported_{index} && !write_{index}"),
            );
            node.update(format!("imported_{index} <= next_imported_{index};"));
        }
        let update = self.update(index, node);
        let input = self.strength_input(&definition.inputs, Value::Next);
        node.wire(4, format!("input_{index}"), input);
        if self.has_side_strength(index) {
            let side = self.strength_input(&definition.sides, Value::Next);
            node.wire(4, format!("side_{index}"), side);
        }
        let rear = match far_input {
            Some(far) => format!("(input_{index} < 4'd15 ? 4'd{far} : input_{index})"),
            None => format!("input_{index}"),
        };
        let desired = if !self.has_side_strength(index) {
            rear
        } else if mode == ComparatorMode::Subtract {
            format!("({rear} < side_{index} ? 4'd0 : {rear} - side_{index})")
        } else {
            format!("({rear} < side_{index} ? 4'd0 : {rear})")
        };
        node.wire(4, format!("desired_{index}"), desired);
        let previous = match self.registers.two_level(index) {
            Some((_, high)) => format!("desired_{index} == 4'd{high}"),
            None => format!("desired_{index}"),
        };
        node.update(format!("previous_input_{index} <= {previous};"));
        self.pending(index, node, None);
        let schedule = and_all(vec![
            update,
            format!("!pending_{index}"),
            format!("desired_{index} != {next_strength}"),
        ]);
        self.schedule(index, node, Some(schedule), None);
    }

    fn lamp(&self, index: usize, node: &mut NodeVerilog) {
        let initial = self.circuit.nodes[index].initial;
        node.register(1, format!("powered_{index}"), bit(initial.powered));
        node.wire(
            1,
            format!("turn_off_{index}"),
            format!("fire_{index} && powered_{index} && !previous_input_{index}"),
        );
        node.wire(
            1,
            format!("fired_powered_{index}"),
            format!("powered_{index} && !turn_off_{index}"),
        );
        if initial.strength != 0 {
            node.register(1, format!("cleared_{index}"), bit(false));
            node.output(
                1,
                format!("next_cleared_{index}"),
                format!("cleared_{index} || turn_off_{index}"),
            );
            node.update(format!("cleared_{index} <= next_cleared_{index};"));
        }
        let update = self.update(index, node);
        self.boolean_input_wire(index, node);
        self.previous_input(index, node, &format!("input_{index}"));
        node.update(format!(
            "powered_{index} <= fired_powered_{index} || {};",
            and_all(vec![update.clone(), format!("input_{index}")])
        ));
        self.pending(index, node, None);
        let schedule = and_all(vec![
            update,
            format!("fired_powered_{index}"),
            format!("!input_{index}"),
            format!("!pending_{index}"),
        ]);
        self.schedule(index, node, Some(schedule), None);
    }

    fn trapdoor(&self, index: usize, node: &mut NodeVerilog) {
        let initial = self.circuit.nodes[index].initial;
        node.register(1, format!("powered_{index}"), bit(initial.powered));
        let update = self.update(index, node);
        self.boolean_input_wire(index, node);
        let mut write = format!("powered_{index} <= input_{index};");
        if self.registers.has_open_register(index) {
            node.register(1, format!("open_{index}"), bit(initial.open));
            write = format!("begin {write} open_{index} <= input_{index}; end");
        }
        let changes = and_all(vec![update, format!("powered_{index} != input_{index}")]);
        node.update(format!("if ({changes}) {write}"));
    }

    fn wire(&self, index: usize, node: &mut NodeVerilog) {
        let initial = self.circuit.nodes[index].initial;
        node.register(
            4,
            format!("strength_{index}"),
            format!("4'd{}", initial.strength),
        );
        let update = self.update(index, node);
        let input = self.strength_input(&self.circuit.nodes[index].inputs, Value::Next);
        node.wire(4, format!("input_{index}"), input);
        node.update(if update == "1'b1" {
            format!("strength_{index} <= input_{index};")
        } else {
            format!("if ({update}) strength_{index} <= input_{index};")
        });
    }

    fn timer(&self, index: usize, node: &mut NodeVerilog) {
        let initial = self.circuit.nodes[index].initial;
        let mut fires = Vec::new();
        for lane in self.registers.lanes(index) {
            let name = lane.name(index);
            let length = self.registers.countdown_length(index, lane);
            let start = if self.registers.holds_initial_tick(index, lane) {
                countdown(initial.delay)
            } else {
                0
            };
            node.vector_register(
                length,
                format!("countdown_{name}"),
                format!("{length}'b{start:0width$b}", width = length as usize),
            );
            fires.push(match self.registers.lane_priority(index, lane) {
                Some(0) => format!("countdown_{name}[0] && firing[0]"),
                Some(priority) => format!("due_{name} && firing[{priority}]"),
                None => format!("due_{name} && (powered_{index} ? firing[1] : firing[2])"),
            });
            if self.registers.lane_priority(index, lane) != Some(0) {
                node.register(1, format!("due_{name}"), bit(false));
            }
        }
        node.wire(1, format!("fire_{index}"), or_all(fires));
    }

    fn pending(&self, index: usize, node: &mut NodeVerilog, pulse: Option<String>) {
        let mut terms: Vec<_> = self
            .registers
            .lanes(index)
            .into_iter()
            .map(|lane| {
                let name = lane.name(index);
                if self.registers.lane_priority(index, lane) == Some(0) {
                    format!("|countdown_{name} && !fire_{index}")
                } else {
                    format!("|countdown_{name} || due_{name} && !fire_{index}")
                }
            })
            .collect();
        terms.extend(pulse);
        node.wire(1, format!("pending_{index}"), terms.join(" || "));
    }

    fn settle_priority(&self, index: usize) -> Option<u8> {
        match self.circuit.nodes[index].kind {
            Kind::Repeater {
                facing_diode: true, ..
            } => Some(0),
            Kind::Repeater { .. } => None,
            kind => timer(kind).and_then(|timer| timer.priority),
        }
    }

    fn schedule(
        &self,
        index: usize,
        node: &mut NodeVerilog,
        settle_schedule: Option<String>,
        fire_schedule: Option<(String, u8)>,
    ) {
        let delay = timer(self.circuit.nodes[index].kind)
            .expect("timed node")
            .delay;
        let mut schedules = Vec::new();
        if let Some(schedule) = settle_schedule {
            node.wire(1, format!("schedule_{index}"), schedule);
            schedules.push((format!("schedule_{index}"), self.settle_priority(index)));
        }
        if let Some((schedule, priority)) = fire_schedule {
            schedules.push((schedule, Some(priority)));
        }
        for lane in self.registers.lanes(index) {
            let name = lane.name(index);
            let sets: Vec<_> = schedules
                .iter()
                .filter_map(|(schedule, priority)| match (lane, priority) {
                    (Lane::Shared, _) => Some(schedule.clone()),
                    (Lane::Priority(lane), Some(priority)) => {
                        (lane == *priority).then(|| schedule.clone())
                    }
                    (Lane::Priority(2), None) => Some(format!("{schedule} && input_{index}")),
                    (Lane::Priority(1), None) => Some(format!("{schedule} && !input_{index}")),
                    (Lane::Priority(_), None) => None,
                })
                .collect();
            let length = self.registers.countdown_length(index, lane);
            let width = length as usize;
            if self.registers.lane_priority(index, lane) != Some(0) {
                node.update(format!(
                    "if (fire_{index}) due_{name} <= 1'b0; else if (begin_tick) due_{name} <= countdown_{name}[0];"
                ));
            }
            node.update(format!(
                "countdown_{name} <= (begin_tick ? countdown_{name} >> 1 : countdown_{name}) | ({} ? {length}'b{:0width$b} : {length}'b0);",
                or_all(sets),
                countdown(delay),
            ));
        }
    }

    fn readback_bits(&self, nodes: &[usize]) -> usize {
        let bits = nodes
            .iter()
            .flat_map(|&index| self.registers.fields(index))
            .map(|field| field.width() as usize)
            .sum();
        readback_bytes(bits) * 8
    }

    fn packed(&self, nodes: &[usize]) -> String {
        let fields: Vec<_> = nodes
            .iter()
            .flat_map(|&index| {
                self.registers
                    .fields(index)
                    .into_iter()
                    .map(move |field| (index, field))
            })
            .collect();
        let bits: usize = fields.iter().map(|(_, field)| field.width() as usize).sum();
        let mut parts: Vec<_> = fields
            .iter()
            .map(|(index, field)| field.register(*index))
            .collect();
        let padding = self.readback_bits(nodes) - bits;
        if padding != 0 {
            parts.push(format!("{padding}'d0"));
        }
        parts.reverse();
        let rows: Vec<_> = parts.chunks(16).map(|row| row.join(", ")).collect();
        format!("{{\n        {}\n    }}", rows.join(",\n        "))
    }
}
