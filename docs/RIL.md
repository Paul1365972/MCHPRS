# Redstone Intermediate Language

RIL is a text form of the graph Redpiler compiles, used to test its passes. A RIL file is a module of circuits, schematics and tests. Each has a global name prefixed with `@`, which can be used once per module and refers back to something defined earlier in the file.

A mistake in a file is reported with its line and column, like `opt/x.ril:2:35: unknown value %nope`. Every circuit must keep the invariants of a compiled circuit: for example, a lamp has no outgoing links and an input has no incoming ones.

## Syntax

### Circuits

```
circuit @<name> {}
```

### Schematics

```
schematic @<name> <path>
```

`path` is a string path relative to the RIL file.

### Tests

`test_args <args>` specifies the default `redpiler_args` used by the tests after it. `args` is a string, and a module has at most one.
It must list the passes to run, like `--passes=coalesce,prune-orphans`, and can add the `/redpiler compile` flags `--io-only`, `--wire-dot-out`, `--illegal-states-out` and `--wire-cross-out`. Every wire in a schematic becomes a node, as without `--optimize`.

`test(@<src>) <test_result>` or `test(@<src>, <redpiler_args>) <test_result>`

`src` names a circuit or schematic. `redpiler_args` is a string. If `redpiler_args` is not specified, it must be specified through `test_args`.

`test_result` is a `circuit`, which `rilc test --update` can fill in. A test passes when its passes turn `src` into a circuit with the same nodes, each with the same type, state, port role and output list. Node order, positions and pending ticks don't count. A failing test shows the line of each node that differs, from the expected circuit (`-`) and from the result (`+`).

```
test_args <args>
test(@<src>) circuit @<test_name> {}
```

### Values

Values are a name given to a component prefixed with `%`. They can include ASCII letters, digits and underscores, and name one component of their circuit. An output list can name components defined later in the circuit.

Examples: `%a`, `%repeater`, `%123`

### Output lists

A component may end with `->` and its outgoing links, in the order the engine updates their targets. Each link names its target and its weight, the power lost along the link, from 0 through 14. `side` marks a link into the side of a repeater or comparator.

`-> [%<name>:<weight>, side %<name>:<weight>, ...]`

For example: `%lever = lever false -> [%a:12, side %12:2]`

## Components

All components have a type, a list of arguments, an optional port marker and an optional output list.

These arguments have a standard type:
- `powered`: boolean
- `power`: integer between 0 and 15

### Ports

Levers, buttons and pressure plates are inputs; lamps, trapdoors and note blocks are outputs; other components are no ports. A component whose port role differs from that is marked `input`, `output` or `internal` (no port) after its arguments, like a wire that `--wire-dot-out` made an output:

```
%dot = wire 0 output
```

### repeater

```
%x = repeater <delay>, <prioritized>, <locked>, <powered>
```

`delay` is an integer between 1 and 4.\
`prioritized` is a boolean.\
`locked` is a boolean.

### torch

```
%x = torch <powered>
```

### comparator

```
%x = comparator <mode>, <far_input>, <prioritized>, <power>
```

`mode` can either be `compare` or `subtract`.\
`far_input` can either be `none` or an integer between 0 and 15.\
`prioritized` is a boolean.

### lamp

```
%x = lamp <powered>
```

### button

```
%x = button <powered>
```

### lever

```
%x = lever <powered>
```

### pressure_plate

```
%x = pressure_plate <powered>
```

### trapdoor

```
%x = trapdoor <powered>
```

### wire

```
%x = wire <power>
```

### constant

```
%x = constant <power>[, <locked>]
```

`locked` is a boolean, `false` if omitted. It holds the lock of a repeater that `ConstantFold` turned into a constant.

### note_block

```
%x = note_block <instrument>, <note>, <powered>
```

`instrument` is one of `harp`, `basedrum`, `snare`, `hat`, `bass`, `flute`, `bell`, `guitar`, `chime`, `xylophone`, `iron_xylophone`, `cow_bell`, `didgeridoo`, `bit`, `banjo`, or `pling`.\
`note` is an integer between 0 and 24.
