# Minecraft High-Performance Redstone Server

[![Build Status](https://github.com/Paul1365972/MCHPRS/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/Paul1365972/MCHPRS/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Discord Banner 2](https://discordapp.com/api/guilds/724072903083163679/widget.png)](https://discord.com/invite/svK9JU7)

A Minecraft 1.20.4 creative server built for redstone. Each 512x512 plot runs on a separate thread, allowing for less lag, more concurrency, and many awesome extra features!

MCHPRS is very different from traditional servers. Because this server is tailored to the use of computational redstone, many things that are a part of Vanilla Minecraft servers don't exist here. That being said, MCHPRS comes with many of its own unique features.

MCHPRS has made it possible to run programs such as [a limited form of Minecraft](https://www.youtube.com/watch?v=-BP7DhHTU-I) on CPUs in Minecraft. To accomplish these speeds, we created [Redpiler](docs/Redpiler.md), the "Redstone Compiler".

[Semantics](docs/Semantics.md) lists the supported components and the differences from vanilla.

## Table of Contents

- [Table of Contents](#table-of-contents)
- [Building](#building)
- [Running](#running)
- [Configuration](#configuration)
- [Documentation](#documentation)
- [Acknowledgments](#acknowledgments)
- [Contributing](#contributing)
- [License](#license)

## Building

If the Rust compiler is not already installed, you can find out how [on their official website](https://www.rust-lang.org/tools/install).

```shell
git clone https://github.com/Paul1365972/MCHPRS.git
cd MCHPRS
cargo build --release
```

Once complete, the optimized executable will be located at `./target/release/mchprs` or `./target/release/mchprs.exe` depending on your operating system.

### Features

| Feature | Adds |
| --- | --- |
| `claims` | `/plot claim`, `/plot auto`, `/plot info` and `/plot visit` |
| `whitelist` | `/whitelist`, and `whitelist = true` in `Config.toml` |

`cargo build --release --features claims,whitelist` enables both.
Prebuilt releases have both enabled, and so does the image the `Dockerfile` builds.

The image runs the server in the volume `/data` and exposes port 25565.

The redstone backends also run without the server: see [Library](docs/Library.md).

## Running

The server reads and writes its files in the working directory.

| Path | Content |
| --- | --- |
| `Config.toml` | The configuration. The server writes the defaults if the file does not exist. |
| `world/plots` | The plots. If `world/plots/pTEMPLATE` exists at startup, new plots start as a copy of it. Before a plot file of an older version is replaced, it is kept as `p<x>,<z>.v<version>`. |
| `world/plots.db` | The plot claims. Requires the [`claims` feature](#features). |
| `world/players` | The player data. |
| `whitelist.json` | The whitelist. The server creates an empty one if the `whitelist` setting is on and the file does not exist. |
| `logs` | The logs. `logs/latest.log` is the log of the current run. At startup the previous one is renamed to `logs/<n>.log`. |
| `schems` | The schematics of `/schematic`. |

MCHPRS has no permissions: every player can use every command and edit every plot.

The environment variable `MCHPRS_LOG` sets the log level, for example `MCHPRS_LOG=debug`. The default is `info`.

`mchprs --version` prints the version and exits.

`cargo run --release -p mchprs_world_exporter -- world <saves>` writes the plots of `world` as a new Minecraft save in the directory `<saves>`.

## Configuration

MCHPRS will generate a `Config.toml` file in the current working directory when starting the server if it does not exist.

A missing key takes its default and an unknown key is ignored.

The following options are available at the toplevel (under no header):
| Field | Description | Default |
| --- | --- |--- |
| `bind_address` | Bind address and port | `0.0.0.0:25565` |
| `motd` | Message of the day | `"Minecraft High Performance Redstone Server"` |
| `chat_format` | How to format chat message interpolating `username` and `message` with curly braces | `<{username}> {message}` |
| `max_players` | Maximum number of simultaneous players | `99999` |
| `view_distance` | Maximal distance (in chunks) between players and loaded chunks | `8` |
| `whitelist` | Whether or not the whitelist (in `whitelist.json`) should be enabled. Requires the [`whitelist` feature](#features). | `false` |
| `schemati` | Mimic the verification and directory layout used by the Open Redstone Engineers [Schemati plugin](https://github.com/OpenRedstoneEngineers/Schemati) | `false` |
| `block_in_hitbox` | Allow placing blocks inside of players (hitbox logic is simplified) | `true` |
| `autosave_interval_seconds` | Seconds until a changed plot is saved. `0` disables autosave. | `300` |

To change the plot size edit the constants defined in [plot/mod.rs](./crates/core/src/plot/mod.rs).

### Command Aliases

The `command_aliases` table maps an alias to a command with its subcommands and flags, without arguments. Both are written as typed in chat, with the leading `/`.

```toml
[command_aliases]
"/rcio" = "/redpiler compile -io"
"//stackair" = "//stack -a"
```

Arguments and flags given to the alias are added to the command: `//stackair 5 north` runs `//stack 5 north -a`.
An alias replaces a built-in command or alias of the same name.
An alias cannot run another alias of this table or of the [built-in aliases](docs/Commands.md#aliases). It can use short names such as `/rp c`.

### Velocity

MCHPRS has no support for player authentication on its own, but supports Velocity modern ip-forwarding.

To use [Velocity](https://papermc.io/software/velocity) ip-forwarding, you must have a Velocity proxy set up and configured. Make sure `player-info-forwarding-mode` is set to `modern` in your Velocity config. Then, append this to your `Config.toml`:

```toml
[velocity]
enabled = true
# This is the secret contained within `forwarding-secret-file` from your velocity config,
# NOT the path to the file.
secret = "<secret>"
```

### Redpiler

| Key | Description | Default |
| --- | --- | --- |
| `redpiler.default_backend` | The [backend](docs/Redpiler.md#backends) of `/redpiler compile`, optionally with flags that are added to its configured flags: `"direct -o"` | `direct` |
| `redpiler.flags.<backend>` | The default [flags](docs/Redpiler.md#flags) of a backend, written as in the command | None |

```toml
[redpiler]
default_backend = "direct"

[redpiler.flags]
direct = "--optimize --io-only"
```

## Documentation

- [Semantics](docs/Semantics.md): supported redstone and the differences from vanilla, with and without Redpiler.
- [Redpiler](docs/Redpiler.md): compiling a plot in game, flags, probes and breakpoints.
- The backends, each with its tick order, passes, flags and limits: [vanilla](docs/backends/Vanilla.md), [direct](docs/backends/Direct.md), [phase](docs/backends/Phase.md).
- [Commands](docs/Commands.md): every command. `/help` lists them in game.
- [Circuits](docs/Circuits.md): the graph Redpiler compiles, the behaviour of its components and the passes.
- [RIL](docs/RIL.md): the text format of a circuit.
- [Library](docs/Library.md): running the backends from a Rust program.
- [Writing a Backend](docs/Writing_a_backend.md): the crates and traits of a backend, and how the server lists it.
- [rilc](docs/rilc.md): the development tool that compiles, runs and compares Redpiler setups.
- [Adding a Block](docs/Adding_a_block.md): the block data generator.

## Acknowledgments
- [@AL1L](https://github.com/AL1L) for his contributions to worldedit and other various features.
- [@DavidGarland](https://github.com/DavidGarland) for a faster and overall better implementation of `get_entry` in the in-memory storage. This simple function runs 30% of the runtime for redstone.

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License
[MIT](https://choosealicense.com/licenses/mit/)
