# Redstone Intermediate Language

## Syntax

### Circuits

```
circuit @<name> {}
backend_circuit(<backend_name>) @<name> {}
```

Backends each have their own printer and parser to suit their own needs

### Schematics

```
schematic @<name> <path>
```

`path` is a string path relative to the RIL file. If the file is stdin, the path is relative to the working directory.

### Tests

`test_args <args>` specifies the default `redpiler_args` used in test blocks. `args` is a string.

`test(@<src>) <test_result>` or `test(@<src>, <redpiler_args>) <test_result>`

`redpiler_args` is a string. If `redpiler_args` is not specified, it must be specified through `test_args`.

`test_result` can either be a `circuit` or `backend_circuit`. `test_result` can be automatically updated with `rilc test --update`.

```
test_args <args>
test(@<src>) circuit @<test_name> {}
test(@<src>) backend_circuit(<backend_name>) @<test_name> {}
```

### Values

Values are a name given to a component prefixed with `%`. They can include any alphanumeric characters.

Examples: `%a`, `%repeater`, `%123`

### Output lists

A component may end with `->` and the links leading out of it, in the order the engine updates their targets. Each link names its target and its weight, the power lost along the link, from 0 through 255; the `clamp-weights` pass removes links with a weight of 15 or more. `side` marks a link into the side of a repeater or comparator.

`-> [%<name>:<weight>, side %<name>:<weight>, ...]`

For example: `%lever = lever false -> [%a:12, side %12:2]`

## Components

All components have a type, a list of arguments and an optional output list.

These arguments have a standard type:
- `powered`: boolean
- `power`: integer between 0 and 15

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
