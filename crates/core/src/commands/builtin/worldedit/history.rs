use crate::{
    commands::{
        context::ExecutionContext,
        define::Command,
        error::{CommandResult, RuntimeError},
        registry::CommandRegistry,
    },
    plot::PlotWorld,
    worldedit::WorldEditUndo,
};
use mchprs_blocks::BlockPos;
use mchprs_schematic::{create_clipboard, paste_clipboard};

pub(super) fn register(registry: &mut CommandRegistry) {
    registry.register(
        Command::new("/undo")
            .alias("undo")
            .permission("worldedit.history.undo")
            .require_plot_ownership()
            .executes(|ctx| exec_history(ctx, true)),
    );

    registry.register(
        Command::new("/redo")
            .alias("redo")
            .permission("worldedit.history.redo")
            .require_plot_ownership()
            .executes(|ctx| exec_history(ctx, false)),
    );
}

fn exec_history(ctx: &mut ExecutionContext<'_>, undo: bool) -> CommandResult<()> {
    let (plot_x, plot_z) = (ctx.world().x, ctx.world().z);
    let player = ctx.player_mut()?;
    let stack = if undo {
        &mut player.worldedit_undo
    } else {
        &mut player.worldedit_redo
    };
    let Some(entry) = stack.last() else {
        return Err(if undo {
            RuntimeError::NoUndoHistory
        } else {
            RuntimeError::NoRedoHistory
        }
        .into());
    };
    if entry.plot_x != plot_x || entry.plot_z != plot_z {
        return Err(if undo {
            RuntimeError::UndoFromDifferentPlot
        } else {
            RuntimeError::RedoFromDifferentPlot
        }
        .into());
    }
    let entry = stack.pop().unwrap();

    let inverse = apply_history(ctx.world_mut(), &entry);
    let player = ctx.player_mut()?;
    if undo {
        player.worldedit_redo.push(inverse);
        ctx.worldedit_message("Undo successful.")
    } else {
        player.worldedit_undo.push(inverse);
        ctx.worldedit_message("Redo successful.")
    }
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
