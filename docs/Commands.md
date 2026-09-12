# Commands

`/help` lists every command, and `/help <command>` shows the syntax and flags of one.
Flags go after all other arguments. Short flags can be combined, and `=false` turns a flag off: `-as`, `-a=false`.

## General

| Command | Alias | Description |
| --- | --- | --- |
| `/help [<command>]` | `//help` | List the commands, or show the syntax and flags of a command. |
| `/rtps [<tps> \| unlimited]` | `/rtps u` | Show the measured tick rate, or set the redstone ticks per second. The default is 10, fractions are allowed, `0` pauses the plot and `unlimited` removes the limit. A redstone tick is two game ticks. |
| `/radvance <ticks>` | `/radv` | Run the given number of redstone ticks without a rate limit. |
| `/worldsendrate [<hertz>]` | `/wsr` | Show or set how often the plot sends block changes to players, from 0 to 1000 times per second. The default is 60, fractions are allowed and `0` disables periodic sends. |
| `/teleport (<position> \| <player>)` | `/tp` | Teleport to coordinates or to a player. |
| `/speed <speed>` | | Set your flying speed, from 0 to 10. |
| `/gamemode (creative \| spectator)` | `/gmc`, `/gmsp`, `/gamemode 1`, `/gamemode 3` | Change your game mode. |
| `/whitelist add <username>` | | Add a player to the whitelist. |
| `/whitelist remove <username>` | | Remove a player from the whitelist. |
| `/version` | | Show the server version. |
| `/stop` | | Stop the server. |

## Plots

| Command | Alias | Description |
| --- | --- | --- |
| `/plot middle` | | Teleport to the center of the current plot. |
| `/plot teleport <location>` | `/p tp` | Teleport to a plot. `<location>` is its two plot coordinates, `x z`. `~` makes a coordinate relative to the current plot. |
| `/plot lock` | | Stay in the current plot when crossing its border. |
| `/plot unlock` | | Undo `/plot lock`. |
| `/plot select` | `/p sel` | Select the entire plot with WorldEdit. |
| `/plot info` | `/p i` | Show the owner of the current plot. |
| `/plot claim` | `/p c` | Claim the current plot if it has no owner. |
| `/plot auto` | `/p a` | Claim a plot without an owner and teleport to it. |
| `/plot visit <username> [<index>]` | `/p v` | Teleport to a plot of a player. `<index>` selects among the player's plots and defaults to 1. |

## WorldEdit

MCHPRS implements a subset of [WorldEdit](https://worldedit.enginehub.org/en/latest/commands/).

### Navigation

| Command | Alias | Description |
| --- | --- | --- |
| `/up [<distance>]` | `/u` | Move up by the given number of blocks, 1 by default, and place glass under you. |
| `/ascend [<levels>]` | `/asc` | Teleport up by the given number of floors, 1 by default. |
| `/descend [<levels>]` | `/desc` | Teleport down by the given number of floors, 1 by default. |

### Selection

| Command | Alias | Description |
| --- | --- | --- |
| `//wand` | | Give yourself a wooden axe that sets the selection corners with left and right click. |
| `//pos1` | `//1` | Set the first corner to your position. |
| `//pos2` | `//2` | Set the second corner to your position. |
| `//hpos1` | `//h1` | Set the first corner to the block you are looking at. |
| `//hpos2` | `//h2` | Set the second corner to the block you are looking at. |
| `//sel` |  | Clear the selection. |
| `//expand [<amount>] [<direction>]` | `//e` | Grow the selection in the direction by `<amount>`. |
| `//contract [<amount>] [<direction>]` | | Shrink the selection in the direction by `<amount>`. |
| `//shift [<amount>] [<direction>]` | | Move the selection without moving its blocks. |

`<amount>` defaults to 1.

### Region

| Command | Alias | Description |
| --- | --- | --- |
| `//set <pattern>` | | Fill the selection with the pattern. |
| `//replace <from> <to>` |  | Replace blocks matching `<from>` with `<to>`. |
| `//count <mask>` | | Count the blocks in the selection that match the mask. |
| `//stack [<count>] [<direction>]` | `//s` | Repeat the selection `<count>` times in the direction. `-a`/`--ignore-air` skips air. |
| `//rstack [<count>] [<direction>] [<offset>]` | `//rs` | Repeat the selection `<count>` times, each copy `<offset>` blocks from the previous one, 2 by default. Copies can overlap, air is skipped and the direction can be diagonal. `-w`/`--with-air` copies air. `-e`/`--expand-selection` selects all copies. |
| `//move [<count>] [<direction>]` | | Move the selected blocks by `<count>` blocks and leave air behind. Blocks moved out of the plot are discarded. `-a`/`--ignore-air` skips air. `-s`/`--shift-selection` moves the selection with the blocks. |
| `//update` | | Update every block in the selection. `-p`/`--plot` updates the entire plot. |
| `//replacecontainer <from> <to>` | `//rc` | Replace containers of type `<from>` in the selection with `<to>` containers that give the same signal strength. A container whose strength `<to>` cannot produce is kept. |

`<count>` defaults to 1.

### Clipboard

| Command | Alias | Description |
| --- | --- | --- |
| `//copy` | `//c` | Copy the selection to the clipboard. |
| `//cut` | `//x` | Copy the selection to the clipboard and replace it with air. |
| `//paste` | `//v` | Paste the clipboard relative to your position. `-a`/`--ignore-air` skips air. `-u`/`--update` updates the pasted blocks. `-s`/`--select-region` selects the pasted region. |
| `//flip [<direction>]` | `//f` | Mirror the clipboard along the direction. |
| `//rotate <angle>` | `//r` | Rotate the clipboard around the vertical axis by a multiple of 90 degrees. |
| `//undo` | `/undo` | Undo the last WorldEdit operation. |
| `//redo` | `/redo` | Redo the last undone WorldEdit operation. |

### Schematics

`/schematic` is also available as `/schem`, `//schematic` and `//schem`.

| Command | Alias | Description |
| --- | --- | --- |
| `//schematic load <filename>` | `//load` | Load a Sponge version 2 or 3 schematic into the clipboard. |
| `//schematic save <filename>` | `//save` | Save the clipboard as a Sponge version 2 schematic. |

Schematics are stored in `./schems/`, or in `./schems/<uuid>/` when [`schemati`](../README.md#configuration) is enabled.
A file name consists of letters, digits, `_`, `-`, `.` and `+`, and ends in `.schem` or `.schematic`.

## Aliases

| Alias | Command |
| --- | --- |
| `/gmc` | `/gamemode creative` |
| `/gmsp` | `/gamemode spectator` |
| `//va` | `//paste -a` |
| `//sa` | `//stack -a` |
| `//load` | `/schematic load` |
| `//save` | `/schematic save` |

Arguments given to an alias are added to its command: `//sa 5 north` runs `//stack 5 north -a`.
A server can define more aliases in [`Config.toml`](../README.md#command-aliases).

## Argument Types

| Type | Examples | Description |
| --- | --- | --- |
| `<direction>` | `me`, `left`/`l`, `right`/`r`, `up`/`u`, `down`/`d`, `north`/`n`, `south`/`s`, `east`/`e`, `west`/`w` | A fixed direction, or one relative to where you look. The default is `me`, the direction you face. |
| `<direction>` of `//rstack` | `leftup`/`lu`, `northdown`/`nd` | Every `<direction>`, and each horizontal direction combined with `up` or `down`. |
| `<type>` | `barrel`, `furnace`, `hopper` | A container type. |
| `<angle>` | `90`, `180`, `270`, `-90` | Degrees, a multiple of 90. |
| `^repeater[delay=3]` | The given block, with the values of the properties it shares with the existing block, and the given properties changed. |
| `=5922` | The block state with this id. |
| `<pattern>` | `stone`, `minecraft:glass`, `stone,20%glass`, `=5922` | Blocks to place in their default state. Commas separate random choices; prefixes such as `20%` set weights (default `100%`). `=` followed by a number selects an exact block state ID. |
| `<mask>` | `stone,glass`, `=5922` | Blocks to match, written like a pattern. Each block name matches only its default state. |
