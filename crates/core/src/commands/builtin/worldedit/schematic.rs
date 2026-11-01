use crate::commands::{
    argument::ArgumentType,
    define::Command,
    error::CommandError,
    registry::CommandRegistry,
    schematic::{save_schematic, schematic_names, schematic_path},
};
use itertools::Itertools;
use std::{fs, io, time::Instant};
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
                                    "There was an error loading the schematic. Check console for more details.",
                                ));
                            }
                        };
                        match mchprs_schematic::read::read(&bytes) {
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
                                    "The schematic was loaded to your clipboard. Do //paste to birth it into the world. ({:?})",
                                    start_time.elapsed()
                                ))
                            }
                            Err(error) => {
                                warn!("There was an error loading a schematic:");
                                warn!("{}", error);
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
                        let bytes = mchprs_schematic::write::write(clipboard).map_err(|error| {
                            warn!("There was an error encoding a schematic: {error}");
                            CommandError::runtime("There was an error saving the schematic.")
                        })?;
                        match save_schematic(&path, &bytes, ctx.flag("force-overwrite")) {
                            Ok(_) => ctx.worldedit_message(&format!(
                                "The schematic was saved successfully. ({:?})",
                                start_time.elapsed()
                            )),
                            Err(error) if error.kind() == io::ErrorKind::AlreadyExists => {
                                Err(CommandError::runtime("File already exists. Use -f flag to overwrite."))
                            }
                            Err(error) => {
                                warn!("There was an error saving a schematic:");
                                warn!("{:?}", error);
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
