use crate::commands::argument::ArgumentType;
use crate::commands::context::ExecutionContext;
use crate::commands::define::{alt, arg, Command};
use crate::commands::error::{CommandResult, RuntimeError};
use crate::commands::registry::CommandRegistry;
use crate::commands::value::PlayerTarget;
use crate::item_stack::ItemStack;
use crate::player::{Gamemode, PlayerPos};
use crate::server::{get_version_string, Message};
use crate::utils::HyphenatedUUID;
use mchprs_blocks::block_entities::ContainerType;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_network::PlayerPacketSender;
use mchprs_redstone::pace::Goal;
use mchprs_save_data::plot_data::{Tps, WorldSendRate};

pub(super) fn register_commands(registry: &mut CommandRegistry) {
    registry.register(
        Command::new("speed")
            .arg("speed", ArgumentType::float(0.0, 10.0))
            .executes(|ctx| {
                let speed: f32 = ctx.arg("speed")?;
                let player = ctx.player_mut()?;
                player.fly_speed = speed;
                player.update_player_abilities();
                let message = format!("Set flying speed to {speed} for {}", player.username);
                ctx.reply(&message)
            }),
    );

    registry.register(
        Command::new("teleport")
            .alias("tp")
            .syntax(alt([
                arg("position", ArgumentType::Vec3),
                arg("player", ArgumentType::PlayerTarget),
            ]))
            .executes(exec_teleport),
    );

    registry.register(
        Command::new("rtps")
            .subcommand(
                Command::new("unlimited")
                    .alias("u")
                    .executes(|ctx| exec_rtps_set(ctx, Tps::Unlimited)),
            )
            .optional("tps", ArgumentType::float(0.0, f32::MAX))
            .executes(|ctx| match ctx.arg_opt::<f32>("tps")? {
                Some(tps) => exec_rtps_set(ctx, Tps::Limited(tps)),
                None => exec_rtps_report(ctx),
            }),
    );

    registry.register(Command::new("stop").executes(|ctx| {
        ctx.plot_mut().send_message(Message::Shutdown);
        ctx.reply("Stopping server...")
    }));

    registry.register(
        Command::new("gamemode")
            .subcommand(
                Command::new("creative")
                    .alias("1")
                    .executes(|ctx| exec_gamemode(ctx, Gamemode::Creative)),
            )
            .subcommand(
                Command::new("spectator")
                    .alias("3")
                    .executes(|ctx| exec_gamemode(ctx, Gamemode::Spectator)),
            ),
    );
    registry.add_custom_alias("gmc", "gamemode creative");
    registry.add_custom_alias("gmsp", "gamemode spectator");

    registry.register(
        Command::new("radvance")
            .alias("radv")
            .subcommand(Command::new("settle").executes(|ctx| exec_radvance(ctx, Goal::Settled)))
            .subcommand(Command::new("stop").executes(|ctx| {
                if ctx.plot_mut().stop_advancing() {
                    return Ok(());
                }
                ctx.reply("The plot isn't advancing.")
            }))
            .arg("ticks", ArgumentType::integer(0, 1_000_000_000))
            .executes(|ctx| {
                let ticks: i32 = ctx.arg("ticks")?;
                exec_radvance(ctx, Goal::Ticks(ticks as u64))
            }),
    );

    registry.register(
        Command::new("worldsendrate")
            .alias("wsr")
            .optional("hertz", ArgumentType::float(0.0, 1000.0))
            .executes(|ctx| {
                if let Some(hertz) = ctx.arg_opt::<f32>("hertz")? {
                    ctx.plot_mut().set_world_send_rate(WorldSendRate(hertz));
                    return ctx.reply("The world send rate was successfully set.");
                }
                let rate = ctx.plot().world_send_rate();
                ctx.reply(&format!("Current world send rate: {} Hz", rate.0))
            }),
    );

    registry.register(Command::new("version").executes(|ctx| ctx.reply(&get_version_string())));

    registry.register(
        Command::new("container")
            .arg("type", ArgumentType::ContainerType)
            .arg("power", ArgumentType::integer(0, 15))
            .executes(|ctx| {
                let container_type: ContainerType = ctx.arg("type")?;
                let power: i32 = ctx.arg("power")?;

                let item = ItemStack::container_with_ss(
                    container_type,
                    SignalStrength::saturating_from(power),
                );
                let player = ctx.player_mut()?;
                let slot = 36 + player.selected_slot;
                player.set_inventory_slot(slot, Some(item));
                ctx.reply("Container created.")
            }),
    );

    #[cfg(feature = "whitelist")]
    registry.register(
        Command::new("whitelist")
            .subcommand(
                Command::new("add")
                    .arg("username", ArgumentType::Word)
                    .executes(|ctx| {
                        let username: String = ctx.arg("username")?;
                        let packet_sender = PlayerPacketSender::new(&ctx.player()?.client);
                        ctx.plot_mut().whitelist_add(username, packet_sender);
                        Ok(())
                    }),
            )
            .subcommand(
                Command::new("remove")
                    .arg("username", ArgumentType::Word)
                    .executes(|ctx| {
                        let username: String = ctx.arg("username")?;
                        let packet_sender = PlayerPacketSender::new(&ctx.player()?.client);
                        ctx.plot_mut().whitelist_remove(username, packet_sender);
                        Ok(())
                    }),
            ),
    );
}

fn exec_teleport(ctx: &mut ExecutionContext<'_>) -> CommandResult<()> {
    if let Some(position) = ctx.position("position")? {
        let PlayerPos { x, y, z } = position;
        if x.abs() > 30_000_000.0 || z.abs() > 30_000_000.0 || y.abs() > 20_000_000.0 {
            return Err(RuntimeError::CoordinateOutOfRange.into());
        }
        ctx.teleport(position)?;
        return ctx.reply(&format!("Teleporting to ({x}, {y}, {z})"));
    }

    let target: PlayerTarget = ctx.arg("player")?;
    let username = match &target {
        PlayerTarget::SelfPlayer => return ctx.reply("Teleported to yourself."),
        PlayerTarget::Name(name) => name.clone(),
        PlayerTarget::Uuid(uuid) => HyphenatedUUID(*uuid).to_string(),
    };
    ctx.reply(&format!("Teleporting to {username}"))?;
    let uuid = ctx.player()?.uuid;
    let player = ctx.plot_mut().leave_plot(uuid);
    ctx.plot_mut()
        .send_message(Message::PlayerTeleportOther(player, target));
    Ok(())
}

fn exec_rtps_set(ctx: &mut ExecutionContext<'_>, tps: Tps) -> CommandResult<()> {
    ctx.plot_mut().set_tps(tps);
    ctx.reply(&format!("The rtps was successfully set to {tps}."))
}

fn exec_rtps_report(ctx: &mut ExecutionContext<'_>) -> CommandResult<()> {
    let report = ctx.plot().tick_rate_report();
    let tps = ctx.plot().tps();
    let rate = |rate: Option<f64>| rate.map_or("–".to_owned(), |rate| format!("{rate:.1}"));
    let message = match report {
        Some(report) => format!(
            "&6RTPS from last 10s, 1m, 5m, 15m: &a{}, {}, {}, {:.1} ({tps})",
            rate(report.ten_seconds),
            rate(report.one_minute),
            rate(report.five_minutes),
            report.fifteen_minutes
        ),
        None => format!("&6No timings data. &a({tps})"),
    };
    ctx.reply_legacy(&message)
}

fn exec_gamemode(ctx: &mut ExecutionContext<'_>, gamemode: Gamemode) -> CommandResult<()> {
    let player_idx = ctx.player_idx();
    ctx.plot_mut().change_player_gamemode(player_idx, gamemode);
    Ok(())
}

fn exec_radvance(ctx: &mut ExecutionContext<'_>, goal: Goal) -> CommandResult<()> {
    let requester = PlayerPacketSender::new(&ctx.player()?.client);
    let plot = ctx.plot_mut();
    if !plot.start_advancing(goal, requester) {
        return ctx.reply("The plot is already advancing. Use /radv stop first.");
    }
    if plot.is_compiling_redpiler() {
        return ctx.reply("The plot will advance once redpiler has compiled.");
    }
    Ok(())
}
