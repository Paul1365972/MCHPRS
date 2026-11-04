use crate::commands::registry::CommandRegistry;
use crate::config::ServerConfig;
#[cfg(feature = "claims")]
use crate::plot::claims::Claims;
use mchprs_save_data::plot_data::PlotData;

pub struct Shared {
    pub config: ServerConfig,
    pub commands: CommandRegistry,
    pub empty_plot: PlotData<'static>,
    #[cfg(feature = "claims")]
    pub claims: Claims,
}
