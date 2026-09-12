use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::CommandResult;
use crate::commands::registry::CommandRegistry;
use mchprs_redpiler::CompilerOptions;
use mchprs_syntax::flag::{Flag, FlagValues};
use std::time::Instant;
use tracing::{debug, warn};

const OPTIMIZE: Flag = Flag::switch("optimize", Some('o'), "Enable redpiler optimizations");
const EXPORT: Flag = Flag::switch("export", Some('e'), "Export the compile graph");
const IO_ONLY: Flag = Flag::switch(
    "io-only",
    Some('i'),
    "Only send block updates of relevant input/output blocks",
);
const UPDATE: Flag = Flag::switch(
    "update",
    Some('u'),
    "Update all blocks after redpiler resets",
);
const WIRE_DOT_OUT: Flag = Flag::switch(
    "wire-dot-out",
    Some('d'),
    "Consider wires in dot shape as output block",
);
const ILLEGAL_STATES_OUT: Flag = Flag::switch(
    "illegal-states-out",
    Some('l'),
    "Consider wires in illegal states as output block",
);
const WIRE_CROSS_OUT: Flag = Flag::switch(
    "wire-cross-out",
    Some('c'),
    "Consider a redstone cross to be an output block",
);
const EXPORT_DOT: Flag = Flag::switch(
    "export-dot",
    None,
    "Create a graphviz dot file of backend graph",
);
const PRINT_AFTER_ALL: Flag = Flag::switch("print-after-all", None, "Print after all passes");
const PRINT_BEFORE_BACKEND: Flag =
    Flag::switch("print-before-backend", None, "Print before backend");

pub(super) fn register_commands(registry: &mut CommandRegistry) {
    let mut compile = Command::new("compile").alias("c");
    let flags = compile.flags_from(&[
        OPTIMIZE,
        EXPORT,
        IO_ONLY,
        UPDATE,
        WIRE_DOT_OUT,
        ILLEGAL_STATES_OUT,
        WIRE_CROSS_OUT,
        EXPORT_DOT,
        PRINT_AFTER_ALL,
        PRINT_BEFORE_BACKEND,
    ]);
    let compile = compile.executes(move |ctx| {
        let flags = ctx.get(flags);
        exec_compile(ctx, flags)
    });
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

fn exec_compile(ctx: &mut ExecutionContext<'_>, flags: FlagValues) -> CommandResult<()> {
    let options = CompilerOptions {
        optimize: flags.is_on(&OPTIMIZE),
        export: flags.is_on(&EXPORT),
        io_only: flags.is_on(&IO_ONLY),
        update: flags.is_on(&UPDATE),
        export_dot_graph: flags.is_on(&EXPORT_DOT),
        wire_dot_out: flags.is_on(&WIRE_DOT_OUT),
        illegal_states_out: flags.is_on(&ILLEGAL_STATES_OUT),
        wire_cross_out: flags.is_on(&WIRE_CROSS_OUT),
        print_after_all: flags.is_on(&PRINT_AFTER_ALL),
        print_before_backend: flags.is_on(&PRINT_BEFORE_BACKEND),
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
    plot.start_redpiler(options);
    let duration = start_time.elapsed();
    plot.publish_world();
    let msg = format!("Compilation completed in {duration:?}");
    debug!(msg);
    ctx.reply(&msg)
}
