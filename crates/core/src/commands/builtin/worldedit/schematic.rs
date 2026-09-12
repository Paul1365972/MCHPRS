use crate::commands::{
    argument::ArgumentType, define::Command, error::CommandError, registry::CommandRegistry,
    schematic::schematic_path,
};
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
                    .executes(|ctx| {
                        let filename: String = ctx.arg("filename")?;
                        let start_time = Instant::now();

                        let path = schematic_path(ctx.player()?.uuid, &filename)?;
                        let clipboard = ctx.get_clipboard()?;
                        match save_schematic(&path, clipboard) {
                            Ok(_) => ctx.worldedit_message(&format!(
                                "The schematic was saved successfully. ({:?})",
                                start_time.elapsed()
                            )),
                            Err(e) => {
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
