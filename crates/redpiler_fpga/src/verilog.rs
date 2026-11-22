use crate::analysis::CircuitAnalysis;
use crate::expr::{register_wire, Expressions};
use crate::layout::{ReadbackScope, StateLayout};
use crate::lower::{lower, CircuitSignal, Instance};
use crate::protocol::Contents;
use crate::rtl::CircuitSource;
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::graph::{LinkKind, SignalKind};
use mchprs_redstone::tick_queue::TickPriority;
use mchprs_simulation::backend::CompileError;
use std::collections::BTreeSet;
use std::fmt::Display;
use strum::VariantArray;

// Measured on BatPU-1: other group sizes cost size or clock.
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
        let analysis = CircuitAnalysis::of(circuit);
        let instances = lower(circuit, &analysis)?;
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
        let expressions = Expressions::new(circuit, &analysis, &instances, &layout);
        let module = module(&instances, &layout, &expressions);
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
    expressions: &Expressions,
) -> String {
    let mut nodes = String::new();
    let mut unread_registers = Vec::new();
    let mut used = BTreeSet::new();
    let mut ticking = 0;
    for (index, instance) in instances.iter().enumerate() {
        let Some(instance) = instance else { continue };
        let copy = ticking / NODES_PER_FIRING_COPY;
        ticking += usize::from(!instance.firing_ports.is_empty());
        let strobes = strobe_ports(instance, copy);
        used.extend(strobes.iter().map(|(_, signal)| signal.clone()));
        let mut ports = vec![("clock".to_owned(), "clock".to_owned())];
        ports.extend(strobes);
        ports.extend(input_ports(instance, NodeId::new(index), expressions));
        ports.extend(register_ports(
            instance,
            index,
            layout,
            &mut unread_registers,
        ));
        nodes.push_str(&format!(
            "    {} #({}) node_{index} ({});\n",
            instance.module,
            connections(&instance.parameters),
            connections(&ports)
        ));
    }

    let copies = ticking.div_ceil(NODES_PER_FIRING_COPY).max(1);
    let firing_bits = copies * TickPriority::ALL.len();
    let declarations = declarations(unread_registers.iter().cloned());
    let driven = CircuitSignal::VARIANTS.iter().map(CircuitSignal::to_string);
    let driven = driven.chain((0..firing_bits).map(firing));
    let unused = driven
        .filter(|signal| !used.contains(signal))
        .chain(unread_registers.into_iter().map(|(register, _)| register));
    let unused = unused_sink(unused.collect());
    let state_tie_off = match layout.holds_no_register() {
        true => "    assign state = 1'b0;\n",
        false => "",
    };
    let taken_valid = CircuitSignal::TakenValid;
    let taken_node = CircuitSignal::TakenNode;
    let taken_input = CircuitSignal::TakenInput;
    let firing_end = firing_bits - 1;
    format!(
        "module circuit (\n\
         `include \"circuit_ports.svh\"\n\
         );\n    \
         logic [{firing_end}:0] firing;\n    \
         logic {taken_valid};\n    \
         logic [pkg::NODE_BITS-1:0] {taken_node};\n    \
         pkg::drive_input_t {taken_input};\n\
         {declarations}{unused}{state_tie_off}    \
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

fn strobe_ports(instance: &Instance, copy: usize) -> Vec<(String, String)> {
    let circuit_signals = instance.circuit_signals.iter();
    let circuit_signals = circuit_signals.map(|signal| (signal.to_string(), signal.to_string()));
    let firing_ports = instance.firing_ports.iter();
    let firing_ports = firing_ports.map(|&(port, priority)| {
        let bit = copy * TickPriority::ALL.len() + priority as usize;
        (port.to_string(), firing(bit))
    });
    circuit_signals.chain(firing_ports).collect()
}

fn input_ports(
    instance: &Instance,
    node: NodeId,
    expressions: &Expressions,
) -> Vec<(String, String)> {
    let inputs = instance.inputs.iter();
    let ports = inputs.map(|&(kind, signal)| {
        let side = match kind {
            LinkKind::Default => "default",
            LinkKind::Side => "side",
        };
        let (input, expression) = match signal {
            SignalKind::OnOff => ("on", expressions.on(node, kind)),
            SignalKind::Strength => ("strength", expressions.strength(node, kind)),
        };
        (format!("{side}_{input}"), expression)
    });
    ports.collect()
}

fn register_ports(
    instance: &Instance,
    index: usize,
    layout: &StateLayout,
    unread_registers: &mut Vec<(String, usize)>,
) -> Vec<(String, String)> {
    let node = NodeId::new(index);
    let ports = instance.all_registers().map(|register| {
        let wire = register_wire(layout, node, register);
        if layout.offset(node, register).is_none() {
            unread_registers.push((wire.clone(), register.width()));
        }
        (register.port().to_owned(), wire)
    });
    ports.collect()
}

fn declarations(signals: impl Iterator<Item = (String, usize)>) -> String {
    let declarations = signals.map(|(signal, width)| match width {
        1 => format!("    logic {signal};\n"),
        width => format!("    logic [{}:0] {signal};\n", width - 1),
    });
    declarations.collect()
}

fn unused_sink(unused: Vec<String>) -> String {
    match unused.is_empty() {
        true => String::new(),
        false => format!(
            "    wire unused = &{{\n        1'b0,\n        {}\n    }};\n",
            unused.join(",\n        ")
        ),
    }
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
