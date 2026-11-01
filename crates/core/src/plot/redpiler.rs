use mchprs_redpiler::compile::CompileOptions;
use mchprs_redpiler::ports::{Observed, Ports};
use mchprs_redpiler::simulation::Simulation;

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct RedpilerOptions {
    pub optimize: bool,
    pub reorder: bool,
    pub io_only: bool,
    pub update: bool,
    pub wire_dot_out: bool,
    pub illegal_states_out: bool,
    pub wire_cross_out: bool,
}

impl RedpilerOptions {
    pub fn compile_options(&self) -> CompileOptions {
        let mut ports = Ports::conventional();
        if self.wire_dot_out {
            ports = ports.with_wire_dots();
        }
        if self.wire_cross_out {
            ports = ports.with_wire_crosses();
        }
        if self.illegal_states_out {
            ports = ports.with_illegal_wires();
        }
        let observed = if self.io_only {
            Observed::Ports
        } else if self.optimize {
            Observed::Components
        } else {
            Observed::Everything
        };
        CompileOptions {
            ports,
            observed,
            optimize: self.optimize,
            reorder: self.reorder,
        }
    }
}

pub struct Redpiler {
    pub options: RedpilerOptions,
    pub simulation: Simulation,
}
