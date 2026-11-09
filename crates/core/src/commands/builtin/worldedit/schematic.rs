use crate::commands::argument_parser::{SchematicFileParser, WordParser};
use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandError, CommandResult};
use crate::commands::registry::CommandRegistry;
use crate::commands::schematic::{save_schematic, schematic_names, schematic_path};
use itertools::Itertools;
use mchprs_syntax::flag::Flag;
use std::time::Instant;
use std::{fs, io};
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

    let bytes = match fs::read(&path) {
        Ok(bytes) => bytes,
        Err(error) if error.kind() == io::ErrorKind::NotFound => {
            return Err(CommandError::runtime(
                "The specified schematic file could not be found.",
            ));
        }
        Err(error) => {
            warn!("There was an error reading a schematic: {error}");
            return Err(CommandError::runtime(
                "There was an error loading the schematic. \
                 Check console for more details.",
            ));
        }
    };
    match mchprs_schematic::read(&bytes) {
        Ok(schematic) => {
            ctx.set_clipboard(schematic.volume)?;
            if !schematic.unknown_blocks.is_empty() {
                ctx.worldedit_message(&format!(
                    "Blocks with unknown or invalid states were replaced with air: {}",
                    schematic
                        .unknown_blocks
                        .iter()
                        .map(|(name, count)| format!("{name} ({count})"))
                        .join(", ")
                ))?;
            }
            ctx.worldedit_message(&format!(
                "The schematic was loaded to your clipboard. \
                 Do //paste to birth it into the world. ({:?})",
                start_time.elapsed()
            ))
        }
        Err(error) => {
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
    let bytes = mchprs_schematic::write(clipboard).map_err(|error| {
        warn!("There was an error encoding a schematic: {error}");
        CommandError::runtime("There was an error saving the schematic.")
    })?;
    match save_schematic(&path, &bytes, force_overwrite) {
        Ok(_) => ctx.worldedit_message(&format!(
            "The schematic was saved successfully. ({:?})",
            start_time.elapsed()
        )),
        Err(error) if error.kind() == io::ErrorKind::AlreadyExists => Err(CommandError::runtime(
            "File already exists. Use -f flag to overwrite.",
        )),
        Err(error) => {
            warn!("There was an error saving a schematic: {error:?}");
            Err(CommandError::runtime(
                "There was an error saving the schematic.",
            ))
        }
    }
}
