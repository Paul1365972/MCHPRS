use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandError, CommandResult};
use crate::commands::registry::CommandRegistry;
use crate::config::Backends;
use mchprs_redpiler_backends::BackendSetup;

pub(super) fn register_commands(registry: &mut CommandRegistry, backends: Backends) {
    let default = backends.default;
    let mut compile = Command::new("compile")
        .alias("c")
        .flags(default.backend().flags())
        .executes(move |ctx| exec_compile(ctx, &default));
    for setup in backends.all {
        compile = compile.subcommand(
            Command::new(setup.backend().name())
                .flags(setup.backend().flags())
                .executes(move |ctx| exec_compile(ctx, &setup)),
        );
    }
    registry.register(
        Command::new("redpiler")
            .alias("rp")
            .subcommand(compile)
            .subcommand(
                Command::new("reset")
                    .alias("r")
                    .flag(
                        'r',
                        "restore",
                        "Return every component to the state it was compiled in",
                    )
                    .flag('u', "update", "Update every block in the plot afterwards")
                    .executes(|ctx| {
                        let restore = ctx.flag("restore");
                        let update = ctx.flag("update");
                        ctx.plot_mut().stop_redpiler(restore, update);
                        ctx.reply("Redpiler has been reset.")
                    }),
            ),
    );
}

fn exec_compile(ctx: &mut ExecutionContext<'_>, setup: &BackendSetup) -> CommandResult<()> {
    let setup = setup.overridden(ctx.flags().cloned().unwrap_or_default());
    let plan = setup.configure().map_err(CommandError::runtime)?;
    let name = setup.backend().name();
    ctx.plot_mut().reset_redpiler();
    ctx.plot_mut()
        .start_redpiler(setup, plan)
        .map_err(CommandError::runtime)?;
    ctx.reply(&format!("Compiling for the {name} backend..."))
}
