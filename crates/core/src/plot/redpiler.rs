use mchprs_commands::flag::Flag;
use mchprs_redpiler::simulation::Simulation;
use mchprs_redpiler_backends::BackendSetup;
use mchprs_redstone::pace::TickRate;
use mchprs_save_data::plot_data::Tps;

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

pub fn tick_rate(tps: Tps) -> TickRate {
    match tps {
        Tps::Limited(tps) => TickRate::per_second(f64::from(tps)),
        Tps::Unlimited => TickRate::Unlimited,
    }
}
