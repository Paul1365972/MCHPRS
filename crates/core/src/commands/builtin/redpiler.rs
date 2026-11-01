use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandError, CommandResult};
use crate::commands::registry::CommandRegistry;
use mchprs_redpiler::CompilerOptions;
use std::time::Instant;
use tracing::{debug, warn};

pub(super) fn register_commands(registry: &mut CommandRegistry) {
    registry.register(
        Command::new("redpiler")
            .alias("rp")
            .subcommand(
                Command::new("compile")
                    .alias("c")
                    .flag('o', "optimize", "Enable redpiler optimizations")
                    .flag(
                        'r',
                        "reorder",
                        "Let optimizations reorder updates within a tick",
                    )
                    .flag(
                        'i',
                        "io-only",
                        "Only send block updates of relevant input/output blocks",
                    )
                    .flag('u', "update", "Update all blocks after redpiler resets")
                    .flag(
                        'd',
                        "wire-dot-out",
                        "Consider wires in dot shape as output block",
                    )
                    .flag(
                        'l',
                        "illegal-states-out",
                        "Consider wires in so-called \"illegal\" states as output block",
                    )
                    .flag(
                        'c',
                        "wire-cross-out",
                        "Consider a redstone cross to be an output block",
                    )
                    .flag(
                        None,
                        "export-dot",
                        "Create a graphviz dot file of backend graph",
                    )
                    .flag(None, "print-after-all", "Print after all passes")
                    .flag(None, "print-before-backend", "Print before backend")
                    .executes(exec_compile),
            )
            .subcommand(Command::new("reset").alias("r").executes(|ctx| {
                let plot = ctx.plot_mut();
                plot.reset_redpiler();
                plot.publish_world();
                ctx.reply("Redpiler has been reset.")
            })),
    );
}

fn exec_compile(ctx: &mut ExecutionContext<'_>) -> CommandResult<()> {
    let options = CompilerOptions {
        optimize: ctx.flag("optimize"),
        reorder: ctx.flag("reorder"),
        io_only: ctx.flag("io-only"),
        update: ctx.flag("update"),
        export_dot_graph: ctx.flag("export-dot"),
        wire_dot_out: ctx.flag("wire-dot-out"),
        illegal_states_out: ctx.flag("illegal-states-out"),
        wire_cross_out: ctx.flag("wire-cross-out"),
        print_after_all: ctx.flag("print-after-all"),
        print_before_backend: ctx.flag("print-before-backend"),
        ..Default::default()
    };

    if options.optimize {
        let msg =
            "Redpiler optimization is highly unstable and can break builds. Use with caution!";
        warn!("{msg}");
        ctx.reply(msg)?;
    }

    let plot = ctx.plot_mut();
    plot.reset_redpiler();
    let start_time = Instant::now();
    plot.start_redpiler(options)
        .map_err(|error| CommandError::runtime(error.to_string()))?;
    let duration = start_time.elapsed();
    plot.publish_world();
    let msg = format!("Compilation completed in {duration:?}");
    debug!(msg);
    ctx.reply(&msg)
}
