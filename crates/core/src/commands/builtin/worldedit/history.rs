use crate::commands::argument::ArgumentType;
use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandResult, RuntimeError};
use crate::commands::registry::CommandRegistry;
use crate::plot::PlotWorld;
use crate::worldedit::WorldEditUndo;
use mchprs_blocks::BlockPos;
use mchprs_schematic::{create_clipboard, paste_clipboard};

pub(super) fn register(registry: &mut CommandRegistry) {
    registry.register(
        Command::new("/undo")
            .alias("undo")
            .optional("times", ArgumentType::integer(1, 100))
            .executes(|ctx| exec_history(ctx, true)),
    );

    registry.register(
        Command::new("/redo")
            .alias("redo")
            .optional("times", ArgumentType::integer(1, 100))
            .executes(|ctx| exec_history(ctx, false)),
    );
}

fn exec_history(ctx: &mut ExecutionContext<'_>, undo: bool) -> CommandResult<()> {
    let times: i32 = ctx.arg_or("times", 1)?;
    let (world, player) = ctx.world_and_player_mut()?;
    let (source, destination) = if undo {
        (&mut player.worldedit_undo, &mut player.worldedit_redo)
    } else {
        (&mut player.worldedit_redo, &mut player.worldedit_undo)
    };

    let mut done = 0;
    let result = loop {
        if done == times {
            break Ok(());
        }
        let Some(entry) = source.last() else {
            break Ok(());
        };
        if entry.plot_x != world.x || entry.plot_z != world.z {
            break Err(if undo {
                RuntimeError::UndoFromDifferentPlot
            } else {
                RuntimeError::RedoFromDifferentPlot
            });
        }
        let entry = source.pop().unwrap();
        destination.push(apply_history(world, &entry));
        done += 1;
    };

    let (verb, past, none_left) = if undo {
        ("Undo", "Undid", RuntimeError::NoUndoHistory)
    } else {
        ("Redo", "Redid", RuntimeError::NoRedoHistory)
    };
    match done {
        0 => return result.and(Err(none_left)).map_err(Into::into),
        1 => ctx.worldedit_message(&format!("{verb} successful."))?,
        n => ctx.worldedit_message(&format!("{past} {n} operations."))?,
    }
    result.map_err(Into::into)
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
