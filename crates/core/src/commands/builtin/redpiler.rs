use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandError, CommandResult};
use crate::commands::registry::CommandRegistry;
use crate::config::CONFIG;
use crate::plot::redpiler::HOST_FLAGS;
use mchprs_redpiler_backends::BackendSetup;
use std::time::Instant;
use tracing::debug;

pub(super) fn register_commands(registry: &mut CommandRegistry) {
    let backends = CONFIG.redpiler.backends();
    let default = backends.default;
    let mut compile = Command::new("compile")
        .alias("c")
        .flags(default.backend().flags())
        .flags(&HOST_FLAGS)
        .executes(move |ctx| exec_compile(ctx, &default));
    for setup in backends.all {
        compile = compile.subcommand(
            Command::new(setup.backend().name())
                .flags(setup.backend().flags())
                .flags(&HOST_FLAGS)
                .executes(move |ctx| exec_compile(ctx, &setup)),
        );
    }
    registry.register(
        Command::new("redpiler")
            .alias("rp")
            .subcommand(compile)
            .subcommand(Command::new("reset").alias("r").executes(|ctx| {
                let plot = ctx.plot_mut();
                plot.reset_redpiler();
                plot.publish_world();
                ctx.reply("Redpiler has been reset.")
            })),
    );
}

fn exec_compile(ctx: &mut ExecutionContext<'_>, setup: &BackendSetup) -> CommandResult<()> {
    let setup = setup.overridden(ctx.flags().cloned().unwrap_or_default());
    let plan = setup.configure().map_err(CommandError::runtime)?;
    let plot = ctx.plot_mut();
    plot.reset_redpiler();
    let start_time = Instant::now();
    plot.start_redpiler(setup, plan)
        .map_err(|error| CommandError::runtime(error.to_string()))?;
    let duration = start_time.elapsed();
    plot.publish_world();
    let msg = format!("Compilation completed in {duration:?}");
    debug!(msg);
    ctx.reply(&msg)
}
