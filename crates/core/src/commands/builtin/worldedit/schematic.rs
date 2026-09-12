use crate::commands::{
    argument::ArgumentType,
    define::Command,
    error::CommandError,
    registry::CommandRegistry,
    schematic::{schematic_names, schematic_path},
};
use itertools::Itertools;
use mchprs_schematic::{load_schematic, save_schematic};
use std::{io, time::Instant};
use tracing::warn;

pub(super) fn register(registry: &mut CommandRegistry) {
    registry.register(
        Command::new("schematic")
            .alias("schem")
            .alias("/schematic")
            .alias("/schem")
            .subcommand(
                Command::new("list")
                    .alias("all")
                    .alias("ls")
                    .executes(|ctx| {
                        let schematics = schematic_names(ctx.player()?.uuid).map_err(|error| {
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
                    }),
            )
            .subcommand(
                Command::new("load")
                    .permission("worldedit.clipboard.load")
                    .require_plot_ownership()
                    .arg("filename", ArgumentType::SchematicFile)
                    .executes(|ctx| {
                        let filename: String = ctx.arg("filename")?;
                        let start_time = Instant::now();
                        let path = schematic_path(ctx.player()?.uuid, &filename)?;

                        match load_schematic(&path) {
                            Ok(clipboard) => {
                                ctx.set_clipboard(clipboard)?;
                                ctx.worldedit_message(&format!(
                                    "The schematic was loaded to your clipboard. Do //paste to birth it into the world. ({:?})",
                                    start_time.elapsed()
                                ))
                            }
                            Err(e) => {
                                if let Some(io_err) = e.downcast_ref::<io::Error>()
                                    && io_err.kind() == io::ErrorKind::NotFound
                                {
                                    return Err(CommandError::runtime(
                                        "The specified schematic file could not be found.",
                                    ));
                                }
                                warn!("There was an error loading a schematic:");
                                warn!("{}", e);
                                Err(CommandError::runtime(
                                    "There was an error loading the schematic. Check console for more details.",
                                ))
                            }
                        }
                    }),
            )
            .subcommand(
                Command::new("save")
                    .permission("worldedit.clipboard.save")
                    .require_plot_ownership()
                    .arg("filename", ArgumentType::Word)
                    .flag('f', "force-overwrite", "Overwrite existing file")
                    .executes(|ctx| {
                        let filename: String = ctx.arg("filename")?;
                        let start_time = Instant::now();

                        let path = schematic_path(ctx.player()?.uuid, &filename)?;
                        let clipboard = ctx.get_clipboard()?;
                        match save_schematic(&path, clipboard, ctx.flag("force-overwrite")) {
                            Ok(_) => ctx.worldedit_message(&format!(
                                "The schematic was saved successfully. ({:?})",
                                start_time.elapsed()
                            )),
                            Err(e) => {
                                if e.downcast_ref::<io::Error>().is_some_and(|error| error.kind() == io::ErrorKind::AlreadyExists) {
                                    return Err(CommandError::runtime("File already exists. Use -f flag to overwrite."));
                                }
                                warn!("There was an error saving a schematic:");
                                warn!("{:?}", e);
                                Err(CommandError::runtime(
                                    "There was an error saving the schematic.",
                                ))
                            }
                        }
                    }),
            ),
    );
    registry.add_custom_alias("/load", "/schematic load");
    registry.add_custom_alias("/save", "/schematic save");
}
