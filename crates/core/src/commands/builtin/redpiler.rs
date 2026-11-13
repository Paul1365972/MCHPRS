use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandError, CommandResult};
use crate::commands::registry::CommandRegistry;
use crate::config::Backends;
use mchprs_backends::Setup;
use mchprs_syntax::flag::{Flag, FlagValues};

const RESTORE: Flag = Flag::switch(
    "restore",
    Some('r'),
    "Return the plot to its state before compiling",
);

pub(super) fn register_commands(registry: &mut CommandRegistry, backends: Backends) {
    let mut compile = compile_command("compile", backends.default).alias("c");
    for setup in backends.all {
        let name = setup.name().to_owned();
        compile = compile.subcommand(compile_command(&name, setup));
    }

    let mut reset = Command::new("reset").alias("r");
    let restore = reset.flag(RESTORE);
    let reset = reset.executes(move |ctx| {
        let restore = ctx.get(restore);
        ctx.plot_mut().stop_redpiler(restore);
        ctx.reply("Redpiler has been reset.")
    });

    registry.register(
        Command::new("redpiler")
            .alias("rp")
            .subcommand(compile)
            .subcommand(reset),
    );
}

fn compile_command(name: &str, setup: Setup) -> Command {
    let mut compile = Command::new(name);
    let flags = compile.flags_from(setup.accepted_flags());
    compile.executes(move |ctx| {
        let flags = ctx.get(flags);
        exec_compile(ctx, &setup, flags)
    })
}

fn exec_compile(
    ctx: &mut ExecutionContext<'_>,
    setup: &Setup,
    flags: FlagValues,
) -> CommandResult<()> {
    let setup = setup.with_flags(flags);
    let name = setup.name().to_owned();
    ctx.plot_mut().stop_redpiler(false);
    ctx.plot_mut()
        .start_redpiler(setup)
        .map_err(|error| CommandError::runtime(error.to_string()))?;
    ctx.reply(&format!("Compiling for the {name} backend..."))
}
