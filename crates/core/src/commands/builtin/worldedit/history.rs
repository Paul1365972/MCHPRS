use crate::{
    commands::{
        argument::ArgumentType,
        context::ExecutionContext,
        define::Command,
        error::{CommandResult, RuntimeError},
        registry::CommandRegistry,
    },
    player::Player,
    plot::PlotWorld,
    worldedit::{create_clipboard, paste_clipboard, WorldEditUndo},
};

pub(super) fn register(registry: &mut CommandRegistry) {
    registry.register(
        Command::new("/undo")
            .alias("undo")
            .permission("worldedit.history.undo")
            .require_plot_ownership()
            .optional("times", ArgumentType::integer(1, 100))
            .executes(|ctx| exec_history(ctx, true)),
    );

    registry.register(
        Command::new("/redo")
            .alias("redo")
            .permission("worldedit.history.redo")
            .require_plot_ownership()
            .optional("times", ArgumentType::integer(1, 100))
            .executes(|ctx| exec_history(ctx, false)),
    );
}

fn exec_history(ctx: &mut ExecutionContext<'_>, undo: bool) -> CommandResult<()> {
    let times: i32 = ctx.arg_or("times", 1)?;
    let (plot_x, plot_z) = (ctx.world().x, ctx.world().z);

    let mut done = 0;
    let result = loop {
        if done == times {
            break Ok(());
        }
        let Some(entry) = history(ctx.player_mut()?, undo).last() else {
            break Ok(());
        };
        if entry.plot_x != plot_x || entry.plot_z != plot_z {
            break Err(if undo {
                RuntimeError::UndoFromDifferentPlot
            } else {
                RuntimeError::RedoFromDifferentPlot
            });
        }
        ctx.world_mut()?;
        let entry = history(ctx.player_mut()?, undo).pop().unwrap();
        let inverse = apply_history(ctx.world_mut()?, &entry);
        history(ctx.player_mut()?, !undo).push(inverse);
        done += 1;
    };

    let (verb, past, none_left) = if undo {
        ("Undo", "Undid", RuntimeError::NoUndoHistory)
    } else {
        ("Redo", "Redid", RuntimeError::NoRedoHistory)
    };
    match done {
        0 => return result.and(Err(none_left)).map_err(Into::into),
        1 => ctx.worldedit_message(&format!("{} successful.", verb))?,
        n => ctx.worldedit_message(&format!("{} {} operations.", past, n))?,
    }
    result.map_err(Into::into)
}

fn history(player: &mut Player, undo: bool) -> &mut Vec<WorldEditUndo> {
    if undo {
        &mut player.worldedit_undo
    } else {
        &mut player.worldedit_redo
    }
}

fn apply_history(world: &mut PlotWorld, entry: &WorldEditUndo) -> WorldEditUndo {
    let inverse_clipboards = entry
        .clipboards
        .iter()
        .map(|clipboard| {
            let region = clipboard.region().translated(entry.pos);
            create_clipboard(world, entry.pos, region.min(), region.max())
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
