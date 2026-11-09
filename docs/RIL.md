# Redstone Intermediate Language

RIL is the text format of the [graph](Circuits.md) that Redpiler compiles.
[rilc](rilc.md) reads and writes it.

A RIL file contains circuits, schematics and tests.
A circuit or schematic has a name prefixed with `@`, which is unique in the file and by which a later test refers to it.
`#` starts a comment that runs to the end of the line.

## Circuits

```
circuit @<name> {
  %<label>[, %<label>…] = <kind>[(<field>, …)] [input|output|internal] [tick(…)] [at[…]] [-> […]]
}
```

Each `%<label>… = …` defines one component. A line break counts as a space.

```
circuit @example {
  %lever = lever(powered) -> [%repeater:0]
  %repeater = repeater(delay=2) tick(delay=1, priority=high) -> [%lamp:0]
  %lamp = lamp
}
```

A label consists of ASCII letters, digits and underscores. A link can name a component that is defined later.
A label made only of digits is not a name: rilc writes a component that has no name by its position in the circuit, counted from 0. A [scenario](rilc.md#scenarios) finds a component by any of its labels.
A circuit must satisfy the [invariants](Circuits.md#invariants).

### Components

| Kind | Fields |
| --- | --- |
| `repeater` | `delay=<1 to 4>` (required), `prioritized`, `locked`, `powered` |
| `comparator` | `mode=<compare\|subtract>` (default `compare`), `power=<0 to 15>` (default 0), `far=<0 to 15>`, `prioritized` |
| `torch`, `lamp`, `lever`, `trapdoor` | `powered` |
| `button` | `press_ticks=<10\|15>` (default 10), `powered` |
| `pressure_plate` | `touched`, `powered` |
| `wire` | `power=<0 to 15>` (default 0) |
| `constant` | `power=<0 to 15>` (default 0), `locked` |
| `note_block` | `instrument=<name>` (default `harp`), `note=<0 to 24>` (default 0), `powered` |

`powered` is the signal strength 15, and `far` is the strength of the container that a comparator reads through a block.
`instrument` is one of `harp`, `basedrum`, `snare`, `hat`, `bass`, `flute`, `bell`, `guitar`, `chime`, `xylophone`, `iron_xylophone`, `cow_bell`, `didgeridoo`, `bit`, `banjo`, `pling`, `zombie`, `skeleton`, `creeper`, `dragon`, `wither_skeleton`, `piglin`, or `custom_head`.

`input`, `output` or `internal` overrides the [port role](Circuits.md#ports-and-observed-nodes) of a component. `internal` is no role.

### Pending Ticks

```
tick(delay=<n>, priority=<normal|high|higher|highest>, order=<n>)
```

`priority` defaults to `normal`. The [invariants](Circuits.md#invariants) limit which components take a tick, and its `delay` and `priority`.
Ticks with the same delay and priority need an `order`, which is the order they were scheduled in. Other ticks take none.

### Blocks

`at[(2,1,0) minecraft:repeater[delay=2,facing=west,locked=false,powered=false]]` lists the blocks a component stands for, each as a position and a block state, separated by commas.

### Links

`-> [%<label>:<weight>, side %<label>:<weight>, …]`

Each entry is a link to the named component with a weight from 0 to 14, and `side` marks a side link.
The order of the entries is the order of the links in the graph.
Two entries of the same kind to the same component are refused, since a graph holds [one link](Circuits.md#links) of each kind from one node into another.

## Schematics

```
schematic @<name> "<path>"
```

`path` is a schematic file, relative to the RIL file.

## Tests

```
test_args "<args>"
test(@<input>) circuit @<name> { … }
test(@<input>, "<args>") circuit @<name> { … }
```

A test runs passes on a circuit or schematic of the file and compares the result with the circuit that follows.
Components are matched by their first name, or by position where they have none, and compared in everything but their blocks. The order of links counts; the order of named components does not.

`args` holds `--passes=<names>`, a comma-separated list of [passes](Circuits.md#passes), and optionally `--io-only`. A schematic also takes `--wire-dot-out`, `--wire-cross-out` and `--illegal-states-out`.
`test_args` sets the arguments of every later test that gives none. A file has at most one.
