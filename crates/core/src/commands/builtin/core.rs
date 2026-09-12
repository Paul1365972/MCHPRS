use crate::commands::argument_parser::WordParser;
use crate::commands::argument_parser::{
    ContainerParser, FloatParser, IntegerParser, PlayerParser, PositionParser,
};
use crate::commands::context::ExecutionContext;
use crate::commands::define::{alt, Command};
use crate::commands::error::CommandResult;
use crate::commands::registry::CommandRegistry;
use crate::commands::value::{PlayerTarget, PositionCoordinates};
use crate::item_stack::ItemStack;
use crate::player::{Gamemode, PlayerPos};
use crate::server::{get_version_string, Message};
use crate::utils::HyphenatedUUID;
use mchprs_blocks::block_entities::ContainerType;
use mchprs_network::PlayerPacketSender;
use mchprs_save_data::plot_data::{Tps, WorldSendRate};
use std::time::Instant;

pub(super) fn register_commands(registry: &mut CommandRegistry) {
    let mut speed = Command::new("speed");
    let speed_value = speed.required("speed", FloatParser::new(0.0, 10.0));
    registry.register(speed.executes(move |ctx| {
        let speed = ctx.get(speed_value);
        let player = ctx.player_mut()?;
        player.fly_speed = speed;
        player.update_player_abilities();
        let message = format!("Set flying speed to {speed} for {}", player.username);
        ctx.reply(&message)
    }));

    let mut teleport = Command::new("teleport").alias("tp");
    let (position_syntax, position) = teleport.argument("position", PositionParser);
    let (player_syntax, player) = teleport.argument("player", PlayerParser);
    teleport.syntax(alt([position_syntax, player_syntax]));
    registry.register(teleport.executes(move |ctx| {
        if let Some(position) = ctx.get(position) {
            return exec_teleport_to_position(ctx, position);
        }
        let player = ctx
            .get(player)
            .expect("teleport takes a position or a player");
        exec_teleport_to_player(ctx, player)
    }));

    let mut rtps = Command::new("rtps").subcommand(
        Command::new("unlimited")
            .alias("u")
            .executes(|ctx| exec_rtps_set(ctx, Tps::Unlimited)),
    );
    let tps = rtps.optional("tps", FloatParser::new(0.0, f32::MAX));
    registry.register(rtps.executes(move |ctx| match ctx.get(tps) {
        Some(tps) => exec_rtps_set(ctx, Tps::Limited(tps)),
        None => exec_rtps_report(ctx),
    }));

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
    registry.alias("gmc", "gamemode creative");
    registry.alias("gmsp", "gamemode spectator");

    let mut radvance = Command::new("radvance").alias("radv");
    let ticks = radvance.required("ticks", IntegerParser::new(0, 1_000_000_000));
    registry.register(radvance.executes(move |ctx| {
        let ticks = ctx.get(ticks);
        exec_radvance(ctx, ticks)
    }));

    let mut world_send_rate = Command::new("worldsendrate").alias("wsr");
    let hertz = world_send_rate.optional("hertz", FloatParser::new(0.0, 1000.0));
    registry.register(world_send_rate.executes(move |ctx| {
        if let Some(hertz) = ctx.get(hertz) {
            ctx.plot_mut().set_world_send_rate(WorldSendRate(hertz));
            return ctx.reply("The world send rate was successfully set.");
        }
        let rate = ctx.plot().world_send_rate();
        ctx.reply(&format!("Current world send rate: {} Hz", rate.0))
    }));

    registry.register(Command::new("version").executes(|ctx| ctx.reply(&get_version_string())));

    let mut container = Command::new("container");
    let container_type = container.required("type", ContainerParser);
    let power = container.required("power", IntegerParser::new(0, 15));
    registry.register(container.executes(move |ctx| {
        let (container_type, power) = (ctx.get(container_type), ctx.get(power));
        exec_container(ctx, container_type, power)
    }));

    {
        let mut add = Command::new("add");
        let added = add.required("username", WordParser);
        let mut remove = Command::new("remove");
        let removed = remove.required("username", WordParser);
        registry.register(
            Command::new("whitelist")
                .subcommand(add.executes(move |ctx| {
                    let username = ctx.get(added);
                    let packet_sender = PlayerPacketSender::new(&ctx.player()?.client);
                    ctx.plot_mut().whitelist_add(username, packet_sender);
                    Ok(())
                }))
                .subcommand(remove.executes(move |ctx| {
                    let username = ctx.get(removed);
                    let packet_sender = PlayerPacketSender::new(&ctx.player()?.client);
                    ctx.plot_mut().whitelist_remove(username, packet_sender);
                    Ok(())
                })),
        );
    }
}

fn exec_teleport_to_position(
    ctx: &mut ExecutionContext<'_>,
    coordinates: PositionCoordinates,
) -> CommandResult<()> {
    let position = ctx.position(coordinates)?;
    let PlayerPos { x, y, z } = position;
    ctx.teleport(position)?;
    ctx.reply(&format!("Teleporting to ({x}, {y}, {z})"))
}

fn exec_teleport_to_player(
    ctx: &mut ExecutionContext<'_>,
    target: PlayerTarget,
) -> CommandResult<()> {
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
    let report = ctx.plot().generate_timings_report();
    let tps = ctx.plot().tps();
    let message = match report {
        Some(report) => format!(
            "&6RTPS from last 10s, 1m, 5m, 15m: &a{:.1}, {:.1}, {:.1}, {:.1} ({tps})",
            report.ten_s, report.one_m, report.five_m, report.fifteen_m
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

fn exec_radvance(ctx: &mut ExecutionContext<'_>, ticks: i32) -> CommandResult<()> {
    let start_time = Instant::now();
    let plot = ctx.plot_mut();
    plot.tickn(ticks as u64);
    plot.publish_world();

    ctx.reply(&format!(
        "Plot has been advanced by {ticks} ticks ({:?})",
        start_time.elapsed()
    ))
}

fn exec_container(
    ctx: &mut ExecutionContext<'_>,
    container_type: ContainerType,
    power: i32,
) -> CommandResult<()> {
    let item = ItemStack::container_with_ss(container_type, power as u8);
    let player = ctx.player_mut()?;
    let slot = 36 + player.selected_slot;
    player.set_inventory_slot(slot, Some(item));
    ctx.reply("Container created.")
}
