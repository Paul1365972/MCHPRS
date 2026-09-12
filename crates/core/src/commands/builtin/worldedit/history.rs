use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandError, CommandResult};
use crate::commands::registry::CommandRegistry;
use crate::plot::PlotWorld;
use crate::worldedit::WorldEditUndo;
use mchprs_blocks::BlockPos;
use mchprs_schematic::{create_clipboard, paste_clipboard};

pub(super) fn register(registry: &mut CommandRegistry) {
    registry.register(
        Command::new("/undo")
            .alias("undo")
            .executes(|ctx| exec_history(ctx, History::Undo)),
    );

    registry.register(
        Command::new("/redo")
            .alias("redo")
            .executes(|ctx| exec_history(ctx, History::Redo)),
    );
}

#[derive(Clone, Copy)]
enum History {
    Undo,
    Redo,
}

fn exec_history(ctx: &mut ExecutionContext<'_>, history: History) -> CommandResult<()> {
    let (world, player) = ctx.world_and_player_mut()?;
    let (source, destination) = match history {
        History::Undo => (&mut player.worldedit_undo, &mut player.worldedit_redo),
        History::Redo => (&mut player.worldedit_redo, &mut player.worldedit_undo),
    };
    let (name, verb) = match history {
        History::Undo => ("undo", "Undo"),
        History::Redo => ("redo", "Redo"),
    };
    let Some(entry) = source.last() else {
        return Err(CommandError::runtime(format!(
            "There is nothing left to {name}."
        )));
    };
    if entry.plot_x != world.x || entry.plot_z != world.z {
        return Err(CommandError::runtime(format!(
            "{verb} is from a different plot"
        )));
    }
    let entry = source.pop().unwrap();
    destination.push(apply_history(world, &entry));
    ctx.worldedit_message(&format!("{verb} successful."))
}

fn apply_history(world: &mut PlotWorld, entry: &WorldEditUndo) -> WorldEditUndo {
    let inverse_clipboards = entry
        .clipboards
        .iter()
        .map(|cb| {
            let offset = BlockPos::new(cb.offset_x, cb.offset_y, cb.offset_z);
            let first_pos = entry.pos - offset;
            let size = BlockPos::new(
                cb.size_x as i32 - 1,
                cb.size_y as i32 - 1,
                cb.size_z as i32 - 1,
            );
            create_clipboard(world, entry.pos, first_pos, first_pos + size)
        })
        .collect();

    for clipboard in &entry.clipboards {
        paste_clipboard(world, clipboard, entry.pos, false);
    }

    WorldEditUndo {
        clipboards: inverse_clipboards,
        pos: entry.pos,
        plot_x: entry.plot_x,
        plot_z: entry.plot_z,
    }
}
