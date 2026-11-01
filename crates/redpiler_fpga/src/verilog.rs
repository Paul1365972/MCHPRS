use crate::expr::{output_wire, Expressions};
use crate::layout::{ReadbackScope, StateLayout};
use crate::lower::{lower, CircuitSignal, Instance};
use crate::protocol::Contents;
use crate::protocol::{DRIVE_INPUT_BITS, DRIVE_NODE_BITS};
use crate::rtl::CircuitSource;
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::graph::{LinkKind, SignalKind};
use mchprs_redstone::tick_queue::TickPriority;
use mchprs_simulation::backend::CompileError;
use std::collections::BTreeSet;
use std::fmt::Display;

const NODES_PER_FIRING_COPY: usize = 64;

#[derive(Debug, Clone)]
pub struct Design {
    pub source: CircuitSource,
    pub layout: StateLayout,
    pub build_nonce: u32,
}

impl Design {
    pub fn new(
        circuit: &Circuit,
        scope: ReadbackScope,
        build_nonce: u32,
    ) -> Result<Design, CompileError> {
        let instances = lower(circuit)?;
        let layout = StateLayout::new(circuit, &instances, scope);
        let shown_bits = layout.bits(Contents::Shown);
        let state_bits = layout.bits(Contents::All);
        let package = format!(
            "package circuit_pkg;\n    \
             localparam int SHOWN_BITS = {shown_bits};\n    \
             localparam int STATE_BITS = {state_bits};\n    \
             localparam logic [31:0] BUILD_NONCE = 32'h{build_nonce:08x};\n\
             endpackage\n"
        );
        let expressions = Expressions::new(circuit, &instances);
        let module = module(&instances, &layout, expressions);
        Ok(Design {
            source: CircuitSource { package, module },
            layout,
            build_nonce,
        })
    }
}

fn module(
    instances: &[Option<Instance>],
    layout: &StateLayout,
    mut expressions: Expressions,
) -> String {
    let mut nodes = String::new();
    let mut wires = Vec::new();
    let mut unread_registers = Vec::new();
    let mut used = BTreeSet::new();
    let mut ticking = 0;
    for (index, instance) in instances.iter().enumerate() {
        let Some(instance) = instance else { continue };
        let node = NodeId::new(index);
        let copy = ticking / NODES_PER_FIRING_COPY;
        ticking += usize::from(!instance.firing_ports.is_empty());
        let circuit_signals = instance.circuit_signals.iter();
        let circuit_signals =
            circuit_signals.map(|&signal| (signal.name().to_owned(), signal.name().to_owned()));
        let firing_ports = instance.firing_ports.iter();
        let firing_ports = firing_ports.map(|&(port, priority)| {
            let bit = copy * TickPriority::ALL.len() + priority as usize;
            (port.name().to_owned(), firing(bit))
        });
        let taken: Vec<_> = circuit_signals.chain(firing_ports).collect();
        used.extend(taken.iter().map(|(_, signal)| signal.clone()));
        let mut ports = vec![("clock".to_owned(), "clock".to_owned())];
        ports.extend(taken);
        for &(kind, signal) in &instance.inputs {
            let side = match kind {
                LinkKind::Default => "default",
                LinkKind::Side => "side",
            };
            ports.push(match signal {
                SignalKind::OnOff => (format!("{side}_on"), expressions.on(node, kind)),
                SignalKind::Strength => {
                    (format!("{side}_strength"), expressions.strength(node, kind))
                }
            });
        }
        for register in instance.all_registers() {
            let mut placed = layout.registers(node).iter();
            let offset = placed.find(|(placed, _)| *placed == register);
            let signal = match (offset, register.width()) {
                (Some((_, offset)), 1) => format!("state[{offset}]"),
                (Some((_, offset)), width) => format!("state[{offset} +: {width}]"),
                (None, width) => {
                    let register = format!("{}_{index}", register.port());
                    unread_registers.push((register.clone(), width));
                    register
                }
            };
            ports.push((register.port().to_owned(), signal));
        }
        let power = instance.power;
        if instance.has_output {
            ports.push((format!("{}_next", power.port()), output_wire(index)));
            wires.push((index, power.width()));
        }
        let parameters: Vec<_> = instance
            .parameters
            .iter()
            .map(|&(parameter, value)| (parameter.name(), value))
            .collect();
        nodes.push_str(&format!(
            "    {} #({}) node_{index} ({});\n",
            instance.module.name(),
            connections(&parameters),
            connections(&ports)
        ));
    }

    let copies = ticking.div_ceil(NODES_PER_FIRING_COPY).max(1);
    let firing_bits = copies * TickPriority::ALL.len();
    let outputs = wires
        .iter()
        .map(|&(node, width)| (output_wire(node), width));
    let declared = outputs.chain(unread_registers.iter().cloned());
    let declarations = declared.map(|(signal, width)| match width {
        1 => format!("    logic {signal};\n"),
        width => format!("    logic [{}:0] {signal};\n", width - 1),
    });
    let declarations: String = declarations.collect();
    let driven = CircuitSignal::ALL
        .map(|signal| signal.name().to_owned())
        .into_iter();
    let driven = driven.chain((0..firing_bits).map(firing));
    let output_is_read = expressions.into_read_outputs();
    let unread = wires.iter().filter(|&&(node, _)| !output_is_read[node]);
    let unused: Vec<_> = driven
        .filter(|signal| !used.contains(signal))
        .chain(unread.map(|&(node, _)| output_wire(node)))
        .chain(unread_registers.into_iter().map(|(register, _)| register))
        .collect();
    let unused = match unused.is_empty() {
        true => String::new(),
        false => format!(
            "    wire unused = &{{\n        1'b0,\n        {}\n    }};\n",
            unused.join(",\n        ")
        ),
    };
    let padding = match layout.holds_no_register() {
        true => "    assign state = 1'b0;\n",
        false => "",
    };
    let taken_valid = CircuitSignal::TakenValid.name();
    let taken_node = CircuitSignal::TakenNode.name();
    let taken_input = CircuitSignal::TakenInput.name();
    let firing_end = firing_bits - 1;
    let node_end = DRIVE_NODE_BITS - 1;
    let input_end = DRIVE_INPUT_BITS - 1;
    format!(
        "module circuit (\n\
         `include \"circuit_ports.svh\"\n\
         );\n    \
         logic [{firing_end}:0] firing;\n    \
         logic {taken_valid};\n    \
         logic [{node_end}:0] {taken_node};\n    \
         logic [{input_end}:0] {taken_input};\n\
         {declarations}{unused}{padding}    \
         sequencer #(.COPIES({copies})) sequencer (.clock(clock), .hold(hold), \
         .run_start(run_start), .run_mode(run_mode), .run_count(run_count), \
         .drive_valid(drive_valid), .drive_node(drive_node), .drive_input(drive_input), \
         .idle(idle), .ticks(tick_count), .paused(paused), .firing(firing), \
         .{taken_valid}({taken_valid}), \
         .{taken_node}({taken_node}), .{taken_input}({taken_input}));\n\
         {nodes}\
         endmodule\n"
    )
}

fn firing(bit: usize) -> String {
    format!("firing[{bit}]")
}

fn connections<N: Display, V: Display>(pairs: &[(N, V)]) -> String {
    let connections = pairs.iter();
    let connections: Vec<_> = connections
        .map(|(name, value)| format!(".{name}({value})"))
        .collect();
    connections.join(", ")
}
