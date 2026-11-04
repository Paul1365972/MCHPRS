use crate::{
    commands::{
        argument::ArgumentType,
        context::ExecutionContext,
        define::{alt, arg, Command},
        error::{CommandError, CommandResult, RuntimeError},
        registry::CommandRegistry,
        value::{PlayerTarget, PlotCoordinates},
    },
    config::ServerConfig,
    item_stack::ItemStack,
    player::{Gamemode, PlayerPos},
    plot::Plot,
    server::{get_version_string, Message},
    utils::HyphenatedUUID,
};
use mchprs_blocks::block_entities::ContainerType;
use mchprs_blocks::signal_strength::SignalStrength;
use mchprs_commands::flag::FlagValues;
use mchprs_network::PlayerPacketSender;
use mchprs_redpiler::backend::Backend;
use mchprs_redpiler_backends::BACKENDS;
use mchprs_save_data::plot_data::{Tps, WorldSendRate};
use mchprs_worker::Goal;

pub(super) fn register_commands(registry: &mut CommandRegistry, config: &ServerConfig) {
    registry.register(
        Command::new("speed")
            .arg("speed", ArgumentType::float(0.0, 10.0))
            .executes(|ctx| {
                let speed: f32 = ctx.arg("speed")?;
                let player = ctx.player_mut()?;
                player.fly_speed = speed;
                player.update_player_abilities();
                let username = player.username.clone();
                ctx.reply(&format!("Set flying speed to {} for {}", speed, username))
            }),
    );

    registry.register(
        Command::new("teleport")
            .alias("tp")
            .syntax(alt([
                arg("position", ArgumentType::Vec3),
                arg("player", ArgumentType::PlayerTarget),
            ]))
            .executes(|ctx| {
                if let Some(position) = ctx.position("position")? {
                    let PlayerPos { x, y, z } = position;
                    if x.abs() > 30_000_000.0 || z.abs() > 30_000_000.0 || y.abs() > 20_000_000.0 {
                        return Err(RuntimeError::CoordinateOutOfRange.into());
                    }
                    ctx.teleport(position)?;
                    return ctx.reply(&format!("Teleporting to ({}, {}, {})", x, y, z));
                }

                let target: PlayerTarget = ctx.arg("player")?;
                let username = match &target {
                    PlayerTarget::SelfPlayer => return ctx.reply("Teleported to yourself."),
                    PlayerTarget::Name(name) => name.clone(),
                    PlayerTarget::Uuid(uuid) => HyphenatedUUID(*uuid).to_string(),
                };
                ctx.reply(&format!("Teleporting to {}", username))?;
                let uuid = ctx.player()?.uuid;
                let player = ctx.plot_mut().leave_plot(uuid);
                ctx.plot_mut()
                    .send_message(Message::PlayerTeleportOther(player, target));
                Ok(())
            }),
    );

    fn exec_rtps_set(ctx: &mut ExecutionContext<'_>, tps: Tps) -> CommandResult<()> {
        ctx.plot_mut().set_tps(tps);
        ctx.reply(&format!("The rtps was successfully set to {}.", tps))
    }

    registry.register(
        Command::new("rtps")
            .subcommand(
                Command::new("unlimited")
                    .alias("u")
                    .executes(|ctx| exec_rtps_set(ctx, Tps::Unlimited)),
            )
            .optional("tps", ArgumentType::float(0.0, f32::MAX))
            .executes(|ctx| {
                if let Some(tps) = ctx.arg_opt::<f32>("tps")? {
                    return exec_rtps_set(ctx, Tps::Limited(tps));
                }

                let report = ctx.plot_mut().tick_rate_report();
                let tps = ctx.plot_mut().tps();
                let rate =
                    |rate: Option<f64>| rate.map_or("–".to_owned(), |rate| format!("{rate:.1}"));
                let message = match report {
                    Some(report) => format!(
                        "&6RTPS from last 10s, 1m, 5m, 15m: &a{}, {}, {}, {:.1} ({})",
                        rate(report.ten_seconds),
                        rate(report.one_minute),
                        rate(report.five_minutes),
                        report.fifteen_minutes,
                        tps
                    ),
                    None => format!("&6No timings data. &a({})", tps),
                };
                ctx.reply_legacy(&message)
            }),
    );

    registry.register(Command::new("stop").executes(|ctx| {
        ctx.plot_mut().send_message(Message::Shutdown);
        ctx.reply("Stopping server...")
    }));

    fn set_gamemode(ctx: &mut ExecutionContext<'_>, gamemode: Gamemode) -> CommandResult<()> {
        let player_idx = ctx.player_index();
        ctx.plot_mut().change_player_gamemode(player_idx, gamemode);
        Ok(())
    }

    registry.register(
        Command::new("gamemode")
            .subcommand(
                Command::new("creative")
                    .alias("1")
                    .executes(|ctx| set_gamemode(ctx, Gamemode::Creative)),
            )
            .subcommand(
                Command::new("spectator")
                    .alias("3")
                    .executes(|ctx| set_gamemode(ctx, Gamemode::Spectator)),
            ),
    );
    registry.add_custom_alias("gmc", "gamemode creative");
    registry.add_custom_alias("gmsp", "gamemode spectator");

    fn exec_radvance(ctx: &mut ExecutionContext<'_>, goal: Goal) -> CommandResult<()> {
        let requester = PlayerPacketSender::new(&ctx.player()?.client);
        let plot = ctx.plot_mut();
        plot.queue_advance(goal, requester);
        if plot.is_compiling_redpiler() {
            return ctx.reply("The plot will advance once redpiler has compiled.");
        }
        Ok(())
    }

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
                let rate = ctx.plot_mut().world_send_rate();
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

    fn teleport_to_plot(
        ctx: &mut ExecutionContext<'_>,
        plot_x: i32,
        plot_z: i32,
    ) -> CommandResult<()> {
        let center = Plot::get_center(plot_x, plot_z);
        if center.0.abs() > 30_000_000.0 || center.1.abs() > 30_000_000.0 {
            return Err(RuntimeError::CoordinateOutOfRange.into());
        }
        ctx.teleport(PlayerPos::new(center.0, 64.0, center.1))?;
        Ok(())
    }

    let plot = Command::new("plot").alias("p");
    #[cfg(feature = "claims")]
    let plot = plot
        .subcommand(Command::new("info").alias("i").executes(|ctx| {
            let (plot_x, plot_z) = ctx.player()?.pos.plot_pos();
            let claims = &ctx.shared().claims;
            if let Some(owner) = claims.plot_owner(plot_x, plot_z) {
                let username = claims.cached_username(owner.clone()).unwrap_or(owner);
                ctx.reply(&format!("Plot owner is: {}", username))
            } else {
                ctx.reply("Plot is not owned by anyone.")
            }
        }))
        .subcommand(Command::new("claim").alias("c").executes(|ctx| {
            let (plot_x, plot_z) = ctx.player()?.pos.plot_pos();
            if ctx.shared().claims.is_claimed(plot_x, plot_z).unwrap() {
                ctx.reply("Plot is already claimed!")
            } else {
                let player_idx = ctx.player_index();
                ctx.plot_mut().claim_plot(plot_x, plot_z, player_idx);
                Ok(())
            }
        }))
        .subcommand(Command::new("auto").alias("a").executes(|ctx| {
            let mut plot = (0, 0);
            while ctx.shared().claims.is_claimed(plot.0, plot.1).unwrap() {
                plot = Plot::get_next_plot(plot.0, plot.1);
            }
            let player_idx = ctx.player_index();
            ctx.plot_mut().claim_plot(plot.0, plot.1, player_idx);
            Ok(())
        }))
        .subcommand(
            Command::new("visit")
                .alias("v")
                .arg("username", ArgumentType::Word)
                .optional("index", ArgumentType::integer(1, 999))
                .executes(|ctx| {
                    let username: String = ctx.arg("username")?;
                    let index = ctx.arg_opt::<i32>("index")?;

                    let plots = ctx.shared().claims.owned_plots(&username);
                    if plots.is_empty() {
                        return ctx.reply(&format!("{} does not own any plots.", username));
                    }

                    let Some(&(plot_x, plot_z)) = plots.get(index.unwrap_or(1) as usize - 1) else {
                        return ctx.reply(&format!("Plot range (1, {}).", plots.len()));
                    };
                    teleport_to_plot(ctx, plot_x, plot_z)?;

                    match index {
                        Some(index) => {
                            ctx.reply(&format!("Teleported to {}'s plot #{}.", username, index))
                        }
                        None => ctx.reply(&format!("Teleported to {}'s plot.", username)),
                    }
                }),
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
                    ctx.reply(&format!("Teleported to plot ({}, {}).", plot_x, plot_z))
                }),
        )
        .subcommand(Command::new("lock").executes(|ctx| {
            let entity_id = ctx.player()?.entity_id;
            if ctx.plot_mut().add_locked_player(entity_id) {
                let world = ctx.world();
                let (x, z) = (world.x, world.z);
                ctx.reply(&format!(
                    "Locked to plot ({}, {}). Use '/p unlock' to unlock.",
                    x, z
                ))
            } else {
                ctx.reply("You are already locked to this plot.")
            }
        }))
        .subcommand(Command::new("unlock").executes(|ctx| {
            let entity_id = ctx.player()?.entity_id;
            if ctx.plot_mut().remove_locked_player(entity_id) {
                ctx.reply("You are now unlocked.")
            } else {
                ctx.reply("You are not locked to this plot.")
            }
        }))
        .subcommand(Command::new("select").alias("sel").executes(|ctx| {
            let region = ctx.world().region();
            let player = ctx.player_mut()?;
            player.worldedit_set_first_pos(region.min());
            player.worldedit_set_second_pos(region.max());
            Ok(())
        })),
    );

    let default_engine = config.redpiler.default_engine();
    let mut compile = Command::new("compile")
        .alias("c")
        .flags(default_engine.flags());
    for &backend in BACKENDS {
        let config_flags = config.redpiler.flags(backend);
        if backend.name() == default_engine.name() {
            let config_flags = config_flags.clone();
            compile =
                compile.executes(move |ctx| exec_redpiler_compile(ctx, backend, &config_flags));
        }
        compile = compile.subcommand(
            Command::new(backend.name())
                .flags(backend.flags())
                .executes(move |ctx| exec_redpiler_compile(ctx, backend, &config_flags)),
        );
    }
    registry.register(
        Command::new("redpiler")
            .alias("rp")
            .subcommand(compile)
            .subcommand(Command::new("export").alias("e").executes(|ctx| {
                let circuit = ctx
                    .plot_mut()
                    .redpiler
                    .as_ref()
                    .and_then(|redpiler| redpiler.circuit.clone());
                let Some(circuit) = circuit else {
                    return ctx.reply("Redpiler is not running.");
                };
                let world = &ctx.plot_mut().world;
                let path = format!("redpiler-p{},{}.circuit", world.x, world.z);
                std::fs::write(&path, circuit.to_bytes()).map_err(|error| {
                    CommandError::runtime(format!("Failed to export the circuit: {error}"))
                })?;
                ctx.reply(&format!("The circuit was exported to {path}."))
            }))
            .subcommand(
                Command::new("reset")
                    .alias("r")
                    .flag(
                        'r',
                        "restore",
                        "Return every component to the state it was compiled in",
                    )
                    .flag('u', "update", "Update every block in the plot afterwards")
                    .executes(|ctx| {
                        let restore = ctx.flag("restore");
                        let update = ctx.flag("update");
                        ctx.plot_mut().stop_redpiler(restore, update);
                        ctx.reply("Redpiler has been reset.")
                    }),
            ),
    );
}

fn exec_redpiler_compile(
    ctx: &mut ExecutionContext<'_>,
    backend: &'static dyn Backend,
    config_flags: &FlagValues,
) -> CommandResult<()> {
    let mut flags = config_flags.clone();
    flags.extend(ctx.flags());
    let plan = backend.configure(&flags).map_err(CommandError::runtime)?;
    ctx.plot_mut().reset_redpiler();
    ctx.plot_mut()
        .start_redpiler(backend, flags, plan)
        .map_err(CommandError::runtime)?;
    ctx.reply(&format!("Compiling for the {} engine...", backend.name()))
}
