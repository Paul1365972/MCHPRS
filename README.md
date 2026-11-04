# Minecraft High-Performance Redstone Server

[![Build Status](https://github.com/MCHPR/MCHPRS/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/MCHPR/MCHPRS/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Discord Banner 2](https://discordapp.com/api/guilds/724072903083163679/widget.png)](https://discord.com/invite/svK9JU7)

A Minecraft 1.20.4 creative server built for redstone. Each 512x512 plot runs on a separate thread, allowing for less lag, more concurrency, and many awesome extra features!

MCHPRS is very different from traditional servers. Because this server is tailored to the use of computational redstone, many things that are a part of Vanilla Minecraft servers don't exist here. That being said, MCHPRS comes with many of its own unique features.

MCHPRS has made it possible to run programs such as [a limited form of Minecraft](https://www.youtube.com/watch?v=-BP7DhHTU-I) on CPUs in Minecraft. To accomplish these speeds, we created [Redpiler](docs/Redpiler.md), the "Redstone Compiler".

## Table of Contents

- [Table of Contents](#table-of-contents)
- [Building](#building)
    - [Features](#features)
- [Configuration](#configuration)
    - [Command Aliases](#command-aliases)
    - [Velocity](#velocity)
    - [Redpiler](#redpiler)
- [Usage](#usage)
- [Acknowledgments](#acknowledgments)
- [Contributing](#contributing)
- [License](#license)

## Building

If the Rust compiler is not already installed, you can find out how [on their official website](https://www.rust-lang.org/tools/install).

```shell
git clone https://github.com/MCHPR/MCHPRS.git
cd MCHPRS
cargo build --release
```

Once complete, the optimized executable will be located at `./target/release/mchprs` or `./target/release/mchprs.exe` depending on your operating system.

### Features

Plot claims and the whitelist are optional:

| Feature | Adds |
| --- | --- |
| `claims` | `/plot claim`, `/plot auto`, `/plot info` and `/plot visit` |
| `whitelist` | The `whitelist` setting and `/whitelist` |

Enable them with `cargo build --release --features claims,whitelist`. Prebuilt releases have every feature enabled.

## Configuration

MCHPRS will generate a `Config.toml` file in the current working directory when starting the server if it does not exist.

The following options are available at the toplevel (under no header):
| Field | Description | Default |
| --- | --- |--- |
| `bind_address` | Bind address and port | `0.0.0.0:25565` |
| `motd` | Message of the day | `"Minecraft High Performance Redstone Server"` |
| `chat_format` | How to format chat message interpolating `username` and `message` with curly braces | `<{username}> {message}` |
| `max_players` | Maximum number of simultaneous players | `99999` |
| `view_distance` | Maximal distance (in chunks) between players and loaded chunks | `8` |
| `whitelist` | Whether or not the whitelist (in `whitelist.json`) should be enabled. Requires the [`whitelist` feature](#features) | `false` |
| `schemati` | Mimic the verification and directory layout used by the Open Redstone Engineers [Schemati plugin](https://github.com/OpenRedstoneEngineers/Schemati) | `false` |
| `block_in_hitbox` | Allow placing blocks inside of players (hitbox logic is simplified) | `true` |
| `autosave_interval_seconds` | Seconds until a changed plot is saved, `0` disables autosave | `300` |

To change the plot size edit the constants defined in [plot/mod.rs](./crates/core/src/plot/mod.rs).
If `world/plots/pTEMPLATE` exists, new plots start as a copy of it.

Set `MCHPRS_LOG` to change the log level, for example `MCHPRS_LOG=debug`.

### Command Aliases

Aliases go in the `command_aliases` table, both sides without the leading `/`:

```toml
[command_aliases]
"rcio" = "redpiler compile -io"
"/stackair" = "/stack {} -a"
```

Arguments replace `{}`, or go at the end without it: `//stackair 5 north` runs `//stack 5 north -a`. An alias can't run another alias.

### Velocity

MCHPRS has no support for player authentication on its own, but supports Velocity modern ip-forwarding.

To use [Velocity](https://papermc.io/software/velocity) ip-forwarding, you must have a Velocity proxy set up and configured. Make sure `player-info-forwarding-mode` is set to `modern` in your Velocity config. Then, set the following in your `Config.toml`:

```toml
[velocity]
enabled = true
# This is the secret contained within `forwarding-secret-file` from your velocity config,
# NOT the path to the file.
secret = "<secret>"
```

### Redpiler

`default_backend` sets the backend `/redpiler compile` uses. The `redpiler.flags` table sets default [flags](docs/Commands.md#compiler-flags) for each backend, written as in the command. Flags given to the command override them.

```toml
[redpiler]
default_backend = "direct"

[redpiler.flags]
direct = "--optimize --io-only"
```

## Usage

See [Commands](docs/Commands.md) for every command, or use `/help` in-game.

## Acknowledgments
- [@AL1L](https://github.com/AL1L) for his contributions to worldedit and other various features.
- [@DavidGarland](https://github.com/DavidGarland) for a faster and overall better implementation of `get_entry` in the in-memory storage. This simple function runs 30% of the runtime for redstone.

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License
[MIT](https://choosealicense.com/licenses/mit/)
