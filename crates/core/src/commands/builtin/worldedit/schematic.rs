use crate::commands::argument::ArgumentType;
use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandError, CommandResult};
use crate::commands::registry::CommandRegistry;
use crate::commands::schematic::schematic_path;
use mchprs_schematic::{load_schematic, save_schematic};
use std::io;
use std::time::Instant;
use tracing::warn;

pub(super) fn register(registry: &mut CommandRegistry) {
    registry.register(
        Command::new("schematic")
            .alias("schem")
            .alias("/schematic")
            .alias("/schem")
            .subcommand(
                Command::new("load")
                    .arg("filename", ArgumentType::SchematicFile)
                    .executes(exec_load),
            )
            .subcommand(
                Command::new("save")
                    .arg("filename", ArgumentType::Word)
                    .executes(exec_save),
            ),
    );
    registry.add_custom_alias("/load", "/schematic load");
    registry.add_custom_alias("/save", "/schematic save");
}

fn exec_load(ctx: &mut ExecutionContext<'_>) -> CommandResult<()> {
    let filename: &str = ctx.arg("filename")?;
    let start_time = Instant::now();
    let path = schematic_path(ctx.player()?.uuid, filename)?;

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

fn exec_save(ctx: &mut ExecutionContext<'_>) -> CommandResult<()> {
    let filename: &str = ctx.arg("filename")?;
    let start_time = Instant::now();

    let path = schematic_path(ctx.player()?.uuid, filename)?;
    let clipboard = ctx.clipboard()?;
    match save_schematic(&path, clipboard) {
        Ok(_) => ctx.worldedit_message(&format!(
            "The schematic was saved successfully. ({:?})",
            start_time.elapsed()
        )),
        Err(error) => {
            warn!("There was an error saving a schematic: {error:?}");
            Err(CommandError::runtime(
                "There was an error saving the schematic.",
            ))
        }
    }
}
