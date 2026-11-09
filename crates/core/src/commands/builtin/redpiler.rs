use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandError, CommandResult};
use crate::commands::registry::CommandRegistry;
use crate::config::Backends;
use crate::plot::stepped::CompilerOptions;
use mchprs_backends::Setup;
use mchprs_syntax::flag::Flag;
use std::time::Instant;

const UPDATE: Flag = Flag::switch(
    "update",
    Some('u'),
    "Update all blocks after redpiler resets",
);

pub(super) fn register_commands(registry: &mut CommandRegistry, backends: Backends) {
    let mut compile = compile_command("compile", backends.default).alias("c");
    for setup in backends.all {
        let name = setup.name().to_owned();
        compile = compile.subcommand(compile_command(&name, setup));
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

fn compile_command(name: &str, setup: Setup) -> Command {
    let mut compile = Command::new(name);
    let flags = compile.flags_from(setup.accepted_flags());
    let update = compile.flag(UPDATE);
    compile.executes(move |ctx| {
        let options = CompilerOptions {
            flags: setup.with_flags(ctx.get(flags)).flags().clone(),
            update: ctx.get(update),
        };
        exec_compile(ctx, &setup, options)
    })
}

fn exec_compile(
    ctx: &mut ExecutionContext<'_>,
    setup: &Setup,
    options: CompilerOptions,
) -> CommandResult<()> {
    let backend = mchprs_backends::backends()
        .into_iter()
        .find(|backend| backend.name() == setup.name())
        .expect("the configured backend is registered");
    let plot = ctx.plot_mut();
    plot.reset_redpiler();
    let started = Instant::now();
    plot.start_redpiler(backend, options)
        .map_err(|error| CommandError::runtime(error.to_string()))?;
    plot.publish_world();
    ctx.reply(&format!("Compilation completed in {:?}", started.elapsed()))
}
