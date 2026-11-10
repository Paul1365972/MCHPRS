# rilc

It is a development tool:

```sh
cargo run --release -p rilc -- <command>
```

| Command | Effect |
| --- | --- |
| `compile <input>` | Write a world or circuit as RIL after the passes of a setup, or as Graphviz DOT with `--dot`. The setup defaults to `direct`. |
| `test <path>` | Run the [tests](RIL.md#tests) of a RIL file, or of every RIL file under a directory. |

## Inputs and Setups

A world is a Sponge schematic (`.schem`, `.schematic`) or a plot file from `world/plots`, which includes its pending ticks.
A circuit is a [RIL](RIL.md) file (`.ril`).

A setup is a backend followed by its flags, written as after `/redpiler compile`: `-b "direct -o -i"`, `-b vanilla`.

- `--passes=<names>` runs the named [passes](Circuits.md#passes) instead of the passes of the backend: `-b "direct --passes=fold,coalesce"`.

## Tests

`rilc test crates/redpiler/ril_tests` runs the tests of the passes. `--update` replaces the expected circuit of each failing test with its result.
