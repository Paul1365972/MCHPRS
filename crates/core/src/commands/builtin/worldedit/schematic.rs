use crate::commands::argument_parser::{SchematicFileParser, WordParser};
use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandError, CommandResult};
use crate::commands::registry::CommandRegistry;
use crate::commands::schematic::{schematic_names, schematic_path};
use itertools::Itertools;
use mchprs_schematic::{load_schematic, save_schematic};
use mchprs_syntax::flag::Flag;
use std::io;
use std::time::Instant;
use tracing::warn;

const FORCE_OVERWRITE: Flag = Flag::switch("force-overwrite", Some('f'), "Overwrite existing file");

pub(super) fn register(registry: &mut CommandRegistry) {
    let mut load = Command::new("load");
    let loaded = load.required("filename", SchematicFileParser);
    let mut save = Command::new("save");
    let saved = save.required("filename", WordParser);
    let force_overwrite = save.flag(FORCE_OVERWRITE);
    registry.register(
        Command::new("schematic")
            .alias("schem")
            .alias("/schematic")
            .alias("/schem")
            .subcommand(
                Command::new("list")
                    .alias("all")
                    .alias("ls")
                    .executes(exec_list),
            )
            .subcommand(load.executes(move |ctx| {
                let filename = ctx.get(loaded);
                exec_load(ctx, &filename)
            }))
            .subcommand(save.executes(move |ctx| {
                let (filename, force_overwrite) = (ctx.get(saved), ctx.get(force_overwrite));
                exec_save(ctx, &filename, force_overwrite)
            })),
    );
    registry.alias("/load", "/schematic load");
    registry.alias("/save", "/schematic save");
}

fn exec_list(ctx: &mut ExecutionContext<'_>) -> CommandResult<()> {
    let schemati = ctx.shared().config.schemati;
    let schematics = schematic_names(schemati, ctx.player()?.uuid).map_err(|error| {
        warn!("Unable to list schematics: {error}");
        CommandError::runtime("Unable to list schematics. Check console for details.")
    })?;

    if schematics.is_empty() {
        return ctx.worldedit_message("No schematics found.");
    }
    const LISTED: usize = 100;
    let mut listed = schematics.iter().take(LISTED).join(", ");
    if schematics.len() > LISTED {
        listed += &format!(" and {} more", schematics.len() - LISTED);
    }
    ctx.worldedit_message(&format!(
        "Available schematics ({}): {listed}",
        schematics.len()
    ))
}

fn exec_load(ctx: &mut ExecutionContext<'_>, filename: &str) -> CommandResult<()> {
    let start_time = Instant::now();
    let path = schematic_path(ctx.shared().config.schemati, ctx.player()?.uuid, filename)?;

    match load_schematic(&path) {
        Ok(clipboard) => {
            ctx.set_clipboard(clipboard)?;
            ctx.worldedit_message(&format!(
                "The schematic was loaded to your clipboard. \
                 Do //paste to birth it into the world. ({:?})",
                start_time.elapsed()
            ))
        }
        Err(error) => {
            if let Some(io_err) = error.downcast_ref::<io::Error>()
                && io_err.kind() == io::ErrorKind::NotFound
            {
                return Err(CommandError::runtime(
                    "The specified schematic file could not be found.",
                ));
            }
            warn!("There was an error loading a schematic: {error}");
            Err(CommandError::runtime(
                "There was an error loading the schematic. \
                 Check console for more details.",
            ))
        }
    }
}

fn exec_save(
    ctx: &mut ExecutionContext<'_>,
    filename: &str,
    force_overwrite: bool,
) -> CommandResult<()> {
    let start_time = Instant::now();

    let path = schematic_path(ctx.shared().config.schemati, ctx.player()?.uuid, filename)?;
    let clipboard = ctx.clipboard()?;
    match save_schematic(&path, clipboard, force_overwrite) {
        Ok(_) => ctx.worldedit_message(&format!(
            "The schematic was saved successfully. ({:?})",
            start_time.elapsed()
        )),
        Err(error) => {
            if error
                .downcast_ref::<io::Error>()
                .is_some_and(|io_err| io_err.kind() == io::ErrorKind::AlreadyExists)
            {
                return Err(CommandError::runtime(
                    "File already exists. Use -f flag to overwrite.",
                ));
            }
            warn!("There was an error saving a schematic: {error:?}");
            Err(CommandError::runtime(
                "There was an error saving the schematic.",
            ))
        }
    }
}
