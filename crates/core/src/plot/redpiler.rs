use mchprs_commands::flag::Flag;
use mchprs_redpiler::simulation::Simulation;
use mchprs_redpiler_backends::BackendSetup;

pub const UPDATE: Flag = Flag::switch(
    "update",
    Some('u'),
    "Update all blocks after redpiler resets",
);

pub const HOST_FLAGS: [Flag; 1] = [UPDATE];

pub struct Redpiler {
    pub(super) setup: BackendSetup,
    pub(super) ports_only: bool,
    pub(super) simulation: Simulation,
}
