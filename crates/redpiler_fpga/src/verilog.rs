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
use std::fmt::Display;
use strum::VariantArray;

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
        let nodes = nodes_module(&instances, &layout, &expressions);
        Ok(Design {
            source: CircuitSource { package, nodes },
            layout,
            build_nonce,
        })
    }
}

fn nodes_module(
    instances: &[Option<Instance>],
    layout: &StateLayout,
    expressions: &Expressions,
) -> String {
    let mut nodes = String::new();
    let mut unread_registers = Vec::new();
    for (index, instance) in instances.iter().enumerate() {
        let Some(instance) = instance else { continue };
        let mut ports = vec![("clock".to_owned(), "clock".to_owned())];
        ports.extend(strobe_ports(instance));
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

    let declarations = declarations(unread_registers.into_iter());
    let state_tie_off = match layout.holds_no_register() {
        true => "    assign state = 1'b0;\n",
        false => "",
    };
    let circuit_signals = CircuitSignal::VARIANTS.iter();
    let circuit_signals = circuit_signals.map(|signal| {
        let port_type = signal.port_type();
        format!("    input {port_type} {signal},\n")
    });
    let circuit_signals: String = circuit_signals.collect();
    let firing_end = TickPriority::ALL.len() - 1;
    format!(
        "module nodes (\n    \
         input logic clock,\n    \
         input logic [{firing_end}:0] firing,\n\
         {circuit_signals}    \
         output logic [circuit_pkg::STATE_BITS-1:0] state\n\
         );\n\
         {declarations}{state_tie_off}{nodes}\
         endmodule\n"
    )
}

fn strobe_ports(instance: &Instance) -> Vec<(String, String)> {
    let circuit_signals = instance.circuit_signals.iter();
    let circuit_signals = circuit_signals.map(|signal| (signal.to_string(), signal.to_string()));
    let firing_ports = instance.firing_ports.iter();
    let firing_ports = firing_ports.map(|&(port, priority)| (port.to_string(), firing(priority)));
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

fn firing(priority: TickPriority) -> String {
    format!("firing[{}]", priority as usize)
}

fn connections<N: Display, V: Display>(pairs: &[(N, V)]) -> String {
    let connections = pairs.iter();
    let connections: Vec<_> = connections
        .map(|(name, value)| format!(".{name}({value})"))
        .collect();
    connections.join(", ")
}
