use crate::commands::argument_parser::IntegerParser;
use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandError, CommandResult};
use crate::commands::registry::CommandRegistry;
use crate::plot::world::PlotWorld;
use crate::worldedit::{RegionSnapshot, WorldEditUndo};

pub(super) fn register(registry: &mut CommandRegistry) {
    let mut undo = Command::new("/undo").alias("undo");
    let times = undo.optional("times", IntegerParser::new(1, 100));
    registry.register(undo.executes(move |ctx| {
        let times = ctx.get(times).unwrap_or(1);
        exec_history(ctx, History::Undo, times)
    }));

    let mut redo = Command::new("/redo").alias("redo");
    let times = redo.optional("times", IntegerParser::new(1, 100));
    registry.register(redo.executes(move |ctx| {
        let times = ctx.get(times).unwrap_or(1);
        exec_history(ctx, History::Redo, times)
    }));
}

#[derive(Clone, Copy)]
enum History {
    Undo,
    Redo,
}

fn exec_history(ctx: &mut ExecutionContext<'_>, history: History, times: i32) -> CommandResult<()> {
    let (world, player) = ctx.world_and_player_mut()?;
    let (source, destination) = match history {
        History::Undo => (&mut player.worldedit_undo, &mut player.worldedit_redo),
        History::Redo => (&mut player.worldedit_redo, &mut player.worldedit_undo),
    };
    let (name, verb, past) = match history {
        History::Undo => ("undo", "Undo", "Undid"),
        History::Redo => ("redo", "Redo", "Redid"),
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
            break Err(CommandError::runtime(format!(
                "{verb} is from a different plot"
            )));
        }
        let entry = source.pop().unwrap();
        destination.push(apply_history(world, &entry));
        done += 1;
    };

    match done {
        0 => {
            result?;
            return Err(CommandError::runtime(format!(
                "There is nothing left to {name}."
            )));
        }
        1 => ctx.worldedit_message(&format!("{verb} successful."))?,
        n => ctx.worldedit_message(&format!("{past} {n} operations."))?,
    }
    result
}

fn apply_history(world: &mut PlotWorld, entry: &WorldEditUndo) -> WorldEditUndo {
    let inverse_snapshots = entry
        .snapshots
        .iter()
        .map(|snapshot| RegionSnapshot::take(world, snapshot.region()))
        .collect();

    for snapshot in &entry.snapshots {
        snapshot.restore(world);
    }

    WorldEditUndo {
        snapshots: inverse_snapshots,
        plot_x: entry.plot_x,
        plot_z: entry.plot_z,
    }
}
