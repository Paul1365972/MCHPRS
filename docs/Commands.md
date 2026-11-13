# Commands

`/help` lists every command, and `/help <command>` shows the syntax and flags of one.
Flags go after all other arguments. Short flags can be combined, and `=false` turns a flag off: `-as`, `-a=false`.

## General

| Command | Alias | Description |
| --- | --- | --- |
| `/help [<command>]` | `//help` | List the commands, or show the syntax and flags of a command. |
| `/rtps [<tps> \| unlimited]` | `/rtps u` | Show the measured tick rate, or set the redstone ticks per second. The default is 10, fractions are allowed, `0` pauses the plot and `unlimited` removes the limit. A redstone tick is two game ticks. |
| `/radvance <ticks>` | `/radv` | Run the given number of redstone ticks without a rate limit. The plot is paused afterwards. |
| `/radvance settle` | `/radv settle` | Run redstone ticks without a rate limit until no tick is pending. The plot is paused afterwards. A backend that cannot tell when no tick is pending refuses it. |
| `/radvance stop` | `/radv stop` | End the running `/radvance`. A plot runs one `/radvance` at a time, and none while Redpiler compiles. |
| `/worldsendrate [<hertz>]` | `/wsr` | Show or set how often the plot sends block changes to players, from 0 to 1000 times per second. The default is 60, fractions are allowed and `0` disables periodic sends. |
| `/teleport (<position> \| <player>)` | `/tp` | Teleport to coordinates or to a player. |
| `/speed <speed>` | | Set your flying speed, from 0 to 10. |
| `/gamemode (creative \| spectator)` | `/gmc`, `/gmsp`, `/gamemode 1`, `/gamemode 3` | Change your game mode. |
| `/container <type> <power>` | | Give yourself a barrel, furnace or hopper that a comparator reads as signal strength `<power>`, from 0 to 1779. A container type cannot produce every strength above 15. |
| `/whitelist add <username>` | | Add a player to the whitelist. |
| `/whitelist remove <username>` | | Remove a player from the whitelist. |
| `/version` | | Show the server version. |
| `/stop` | | Stop the server. |

`/whitelist` requires the [`whitelist` feature](../README.md#features) and the [`whitelist` setting](../README.md#configuration).

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

`/plot info`, `/plot claim`, `/plot auto` and `/plot visit` require the [`claims` feature](../README.md#features).

## WorldEdit

MCHPRS implements a subset of [WorldEdit](https://worldedit.enginehub.org/en/latest/commands/).

### Navigation

| Command | Alias | Description |
| --- | --- | --- |
| `/jumpto` | `/j` | Teleport onto the block you are looking at. |
| `/unstuck` | `/!` | Move up to the nearest space of two air blocks. |
| `/up [<distance>]` | `/u` | Move up by the given number of blocks, 1 by default, and place glass under you if there is air. `-f`/`--force-flight` enables flight instead of placing glass. `-g`/`--force-glass` places glass even with `-f`. |
| `/ascend [<levels>]` | `/asc` | Teleport up by the given number of floors, 1 by default. |
| `/descend [<levels>]` | `/desc` | Teleport down by the given number of floors, 1 by default. |

### Selection

| Command | Alias | Description |
| --- | --- | --- |
| `//wand` | | Give yourself a wooden axe that sets the selection corners with left and right click. |
| `//pos1 [<coordinates>]` | `//1` | Set the first corner to the coordinates, or to your position. |
| `//pos2 [<coordinates>]` | `//2` | Set the second corner to the coordinates, or to your position. |
| `//hpos1` | `//h1` | Set the first corner to the block you are looking at. |
| `//hpos2` | `//h2` | Set the second corner to the block you are looking at. |
| `//pos [<pos1> [<pos2>]]` | | Set one or both corners. Without arguments, set the first corner to your position. |
| `//sel` | `/;`, `//desel`, `//deselect` | Clear the selection. |
| `//expand [<amount>] [<reverseAmount>] [<direction>]` | `//e` | Grow the selection in the direction by `<amount>`, and in the opposite direction by `<reverseAmount>`. |
| `//expand vert` | `//e vert` | Grow the selection to the full height of the world. |
| `//contract [<amount>] [<reverseAmount>] [<direction>]` | | Shrink the selection by `<amount>` on the side opposite the direction, and by `<reverseAmount>` on the side of the direction. |
| `//shift [<amount>] [<direction>]` | | Move the selection without moving its blocks. |

`<amount>` defaults to 1 and `<reverseAmount>` to 0.

### Region

| Command | Alias | Description |
| --- | --- | --- |
| `//set <pattern>` | | Fill the selection with the pattern. |
| `//replace [<mask>] <pattern>` | `//re`, `//rep` | Replace the blocks that match the mask with the pattern. Without a mask, every block except air is replaced. |
| `//count <mask>` | | Count the blocks in the selection that match the mask. |
| `//distr` | | Show the count and share of each block in the selection. `-c`/`--clipboard` reads the clipboard instead. |
| `//stack [<count>] [<direction>]` | `//s` | Repeat the selection `<count>` times in the direction. `-a`/`--ignore-air` skips air. `-s`/`--shift-selection` selects the last copy. |
| `//rstack [<count>] [<direction>] [<offset>]` | `//rs` | Repeat the selection `<count>` times, each copy `<offset>` blocks from the previous one, 2 by default. Copies can overlap, air is skipped and the direction can be diagonal. `-w`/`--with-air` copies air. `-s`/`--shift-selection` selects the last copy. `-e`/`--expand-selection` selects all copies. |
| `//move [<count>] [<direction>]` | | Move the selected blocks by `<count>` blocks and leave air behind. Blocks moved out of the plot are discarded. `-a`/`--ignore-air` skips air. `-s`/`--shift-selection` moves the selection with the blocks. |
| `//update` | | Update every block in the selection. `-p`/`--plot` updates the entire plot. |
| `//replacecontainer [<from>] <to>` | `//rc` | Replace the barrels, furnaces and hoppers in the selection with containers of type `<to>` that give the same signal strength. `<from>` restricts the replacement to one type. A container whose strength `<to>` cannot produce is kept. |

`<count>` defaults to 1.

### Clipboard

| Command | Alias | Description |
| --- | --- | --- |
| `//copy` | `//c` | Copy the selection to the clipboard. |
| `//cut` | `//x` | Copy the selection to the clipboard and replace it with air. |
| `//paste` | `//v` | Paste the clipboard relative to your position. `-a`/`--ignore-air` skips air. `-u`/`--update` updates the pasted blocks. `-s`/`--select-region` selects the pasted region. `-n`/`--no-paste` selects the region without pasting. |
| `//flip [<direction>]` | `//f` | Mirror the clipboard along the direction. |
| `//rotate <angle>` | `//r` | Rotate the clipboard around the vertical axis by a multiple of 90 degrees. |
| `//undo [<times>]` | `/undo` | Undo the last WorldEdit operations, 1 by default. |
| `//redo [<times>]` | `/redo` | Redo the last undone WorldEdit operations, 1 by default. |

### Schematics

`/schematic` is also available as `/schem`, `//schematic` and `//schem`.

| Command | Alias | Description |
| --- | --- | --- |
| `/schematic list` | `/schematic all`, `/schematic ls` | List the schematic files. |
| `/schematic load <filename>` | `//load` | Load a Sponge version 2 or 3 schematic into the clipboard. |
| `/schematic save <filename>` | `//save` | Save the clipboard as a Sponge version 2 schematic. `-f`/`--force-overwrite` overwrites an existing file. |

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
| `<position>`, `<coordinates>`, `<pos1>`, `<pos2>` | `10 64 -3`, `~ ~5 ~`, `^ ^ ^2` | Three coordinates: absolute, relative to your position with `~`, or to your left, up and forward with `^`. |
| `<type>` | `barrel`, `furnace`, `hopper` | A container type. |
| `<angle>` | `90`, `180`, `270`, `-90` | Degrees, a multiple of 90. |
| `<pattern>` | `stone`, `minecraft:glass`, `repeater[delay=3]`, `stone,20%glass`, `^[powered=true]` | The blocks to place. |
| `<mask>` | `stone,glass`, `repeater[delay=2]`, `!stone`, `#existing`, `>stone`, `%50` | The blocks to match. |

### Patterns

| Pattern | Places |
| --- | --- |
| `stone` | The block in its default state. |
| `repeater[delay=3]` | The block with the given properties. |
| `=5922` | The block state with this id. |
| `stone,glass` | One of the listed blocks, chosen at random per position. |
| `stone,20%glass` | As above, with a weight per block. A block without a weight has weight 1. |
| `^[powered=true]` | The existing block with the given properties changed. Blocks without these properties are kept unchanged. |
| `^repeater[delay=3]` | The given block, with the values of the properties it shares with the existing block, and the given properties changed. |

### Masks

| Mask | Matches |
| --- | --- |
| `stone,glass` | Any of the listed blocks, in any state. |
| `repeater[delay=2,powered=true]` | The block with the given properties. |
| `=5922` | The block state with this id. |
| `!stone` | Every block the following mask does not match. |
| `#existing` | Every block except air. |
| `#solid` | Every solid block. |
| `#surface`, `#exposed` | Every block except air that has air next to it. |
| `>stone` | Blocks above a block that matches. |
| `<stone` | Blocks below a block that matches. |
| `%50` | The given percentage of blocks, chosen at random. |
| `^[delay=2]` | Blocks whose property has this value, and blocks without the property. |
| `^=[delay=2]` | Blocks whose property has this value. |
| `"stone #solid"` | Blocks that match every mask in the quotes. |

A pattern or mask that contains spaces is written in single or double quotes.
Inside quotes, `\` escapes the quote character and `\` itself.
