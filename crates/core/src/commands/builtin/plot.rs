use crate::commands::argument::ArgumentType;
use crate::commands::context::ExecutionContext;
use crate::commands::define::Command;
use crate::commands::error::{CommandResult, RuntimeError};
use crate::commands::registry::CommandRegistry;
use crate::commands::value::PlotCoordinates;
use crate::player::PlayerPos;
use crate::plot::Plot;

pub(super) fn register_commands(registry: &mut CommandRegistry) {
    let plot = Command::new("plot").alias("p");
    #[cfg(feature = "claims")]
    let plot = plot
        .subcommand(Command::new("info").alias("i").executes(exec_info))
        .subcommand(Command::new("claim").alias("c").executes(exec_claim))
        .subcommand(Command::new("auto").alias("a").executes(exec_auto))
        .subcommand(
            Command::new("visit")
                .alias("v")
                .arg("username", ArgumentType::Word)
                .optional("index", ArgumentType::integer(1, 999))
                .executes(exec_visit),
        );
    registry.register(
        plot.subcommand(Command::new("middle").executes(|ctx| {
            let (plot_x, plot_z) = ctx.player()?.pos.plot_pos();
            teleport_to_plot(ctx, plot_x, plot_z)?;
            ctx.reply("Teleported to plot middle.")
        }))
        .subcommand(
            Command::new("teleport")
                .alias("tp")
                .arg("location", ArgumentType::PlotPos)
                .executes(|ctx| {
                    let location: PlotCoordinates = ctx.arg("location")?;
                    let current_plot_pos = ctx.player()?.pos.plot_pos();
                    let (plot_x, plot_z) = location.resolve(current_plot_pos)?;
                    teleport_to_plot(ctx, plot_x, plot_z)?;
                    ctx.reply(&format!("Teleported to plot ({plot_x}, {plot_z})."))
                }),
        )
        .subcommand(Command::new("lock").executes(exec_lock))
        .subcommand(Command::new("unlock").executes(|ctx| {
            let entity_id = ctx.player()?.entity_id;
            if ctx.plot_mut().remove_locked_player(entity_id) {
                ctx.reply("You are now unlocked.")
            } else {
                ctx.reply("You are not locked to this plot.")
            }
        }))
        .subcommand(Command::new("select").alias("sel").executes(|ctx| {
            let region = ctx.plot().world.region();
            let player = ctx.player_mut()?;
            player.worldedit_set_first_pos(region.min());
            player.worldedit_set_second_pos(region.max());
            Ok(())
        })),
    );
}

#[cfg(feature = "claims")]
fn exec_info(ctx: &mut ExecutionContext<'_>) -> CommandResult<()> {
    let (plot_x, plot_z) = ctx.player()?.pos.plot_pos();
    let claims = &ctx.shared().claims;
    if let Some(owner) = claims.plot_owner(plot_x, plot_z) {
        let username = claims.cached_username(&owner).unwrap_or(owner);
        ctx.reply(&format!("Plot owner is: {username}"))
    } else {
        ctx.reply("Plot is not owned by anyone.")
    }
}

#[cfg(feature = "claims")]
fn exec_claim(ctx: &mut ExecutionContext<'_>) -> CommandResult<()> {
    let (plot_x, plot_z) = ctx.player()?.pos.plot_pos();
    if ctx.shared().claims.is_claimed(plot_x, plot_z).unwrap() {
        ctx.reply("Plot is already claimed!")
    } else {
        let player_idx = ctx.player_idx();
        ctx.plot_mut().claim_plot(plot_x, plot_z, player_idx);
        Ok(())
    }
}

#[cfg(feature = "claims")]
fn exec_auto(ctx: &mut ExecutionContext<'_>) -> CommandResult<()> {
    let mut plot = (0, 0);
    while ctx.shared().claims.is_claimed(plot.0, plot.1).unwrap() {
        plot = Plot::get_next_plot(plot.0, plot.1);
    }
    let player_idx = ctx.player_idx();
    ctx.plot_mut().claim_plot(plot.0, plot.1, player_idx);
    Ok(())
}

#[cfg(feature = "claims")]
fn exec_visit(ctx: &mut ExecutionContext<'_>) -> CommandResult<()> {
    let username: &str = ctx.arg("username")?;
    let index = ctx.arg_opt::<i32>("index")?;

    let plots = ctx.shared().claims.owned_plots(username);
    if plots.is_empty() {
        return ctx.reply(&format!("{username} does not own any plots."));
    }

    let Some(&(plot_x, plot_z)) = plots.get(index.unwrap_or(1) as usize - 1) else {
        return ctx.reply(&format!("Plot range (1, {}).", plots.len()));
    };
    teleport_to_plot(ctx, plot_x, plot_z)?;

    match index {
        Some(index) => ctx.reply(&format!("Teleported to {username}'s plot #{index}.")),
        None => ctx.reply(&format!("Teleported to {username}'s plot.")),
    }
}

fn exec_lock(ctx: &mut ExecutionContext<'_>) -> CommandResult<()> {
    let entity_id = ctx.player()?.entity_id;
    if ctx.plot_mut().add_locked_player(entity_id) {
        let world = &ctx.plot().world;
        let (x, z) = (world.x, world.z);
        ctx.reply(&format!(
            "Locked to plot ({x}, {z}). Use '/p unlock' to unlock."
        ))
    } else {
        ctx.reply("You are already locked to this plot.")
    }
}

fn teleport_to_plot(ctx: &mut ExecutionContext<'_>, plot_x: i32, plot_z: i32) -> CommandResult<()> {
    let center = Plot::get_center(plot_x, plot_z);
    if center.0.abs() > 30_000_000.0 || center.1.abs() > 30_000_000.0 {
        return Err(RuntimeError::CoordinateOutOfRange.into());
    }
    ctx.teleport(PlayerPos::new(center.0, 64.0, center.1))?;
    Ok(())
}
