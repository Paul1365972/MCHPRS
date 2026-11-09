# rilc

It is a development tool:

```sh
cargo run --release -p rilc -- <command>
```

| Command | Effect |
| --- | --- |
| `compile <input>` | Write a world or circuit as RIL after the passes of a setup, or as Graphviz DOT with `--dot`. The setup defaults to `direct`. |
| `run <scenario> -b <setup>` | Play a scenario of a world or a RIL circuit on a setup, the way the server runs it, and print the ticks per second. |
| `diff <scenario> -b <setup> -b <setup>` | Play a scenario on two setups and print the first tick after which a component that both show differs. |
| `test <path>` | Run the [tests](RIL.md#tests) of a RIL file, or of every RIL file under a directory. |

`rilc <command> --help` lists the options of a command, and for `compile`, `run` and `diff` also the backends and their flags.

## Inputs and Setups

A world is a Sponge schematic (`.schem`, `.schematic`) or a plot file from `world/plots`, which includes its pending ticks.
A circuit is a [RIL](RIL.md) file (`.ril`).

A setup is a backend followed by its flags, written as after `/redpiler compile`: `-b "direct -o -i"`, `-b vanilla`.

- `--passes=<names>` runs the named [passes](Circuits.md#passes) instead of the passes of the backend: `-b "direct --passes=fold,coalesce"`.

## Scenarios

A scenario is a TOML file that names an input and the inputs to drive.
`run` and `diff` take one.
`run` takes a scenario of a world or a RIL circuit. It starts the setup running by itself and times each step from the command that runs its ticks until the simulation reports them run.
`run --repeat <n>` plays it n times, each on a fresh engine, and `run --save-outputs <file>` writes the final state of every output port to the file, one line per position or, in a RIL circuit, per label.
`run --unlimited <seconds>` then runs the setup without a tick limit for that long and prints that rate too. It is the rate of `/rtps unlimited`.

```toml
input = "alu.schem"
settle = 1000

[[step]]
toggle = [0, 1, 8]
run = 100

[[step]]
set = [4, 1, 2]
power = 15
run = 20
```

| Key | Description |
| --- | --- |
| `input` | The world or RIL file, relative to the scenario file. |
| `circuit` | The circuit to use, if the RIL file holds more than one. |
| `settle` | Worlds only. Update every block and run ticks until none is pending before the scenario starts. The value is the maximum number of ticks. |
| `[[step]]` | A step. It applies at most one action and then runs ticks. Steps that run 0 ticks apply several actions between two ticks. |

| Step key | Description |
| --- | --- |
| `run` | The number of redstone ticks to run after the action. Required. |
| `toggle` | Toggle the input at the target. |
| `press` | Press the button at the target. |
| `set` with `power` | Set the input at the target to the signal strength `power`. |
| `plate` with `power` | Set whether an entity touches the pressure plate at the target. `power` is 0 or 15. |

[Inputs](Circuits.md#inputs) lists the actions each kind of input takes.

A target in a world is a position `[x, y, z]`. A schematic is moved so that its lowest corner is at `[0, 0, 0]`.
A target in a RIL circuit is a label such as `"reset"` or a position of the node.

## Tests

`rilc test crates/redpiler/ril_tests` runs the tests of the passes. `--update` replaces the expected circuit of each failing test with its result.
