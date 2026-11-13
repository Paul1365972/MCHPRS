# rilc

rilc compiles, runs, compares, tests and fuzzes Redpiler setups outside the server.
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
| `fuzz (engine \| passes \| worlds)` | Check a property on random cases. |
| `update <input> <output>` | Update every block of a world, run it until no tick is pending and write it as a schematic. |

`rilc <command> --help` lists the options of a command, and for `compile`, `run` and `diff` also the backends and their flags.

rilc exits with 0 if the claim of the command holds, with 1 if it does not (a test fails, two setups differ, a fuzzed property fails or no case checked it), and with 2 if it cannot run.

## Inputs and Setups

A world is a Sponge schematic (`.schem`, `.schematic`) or a plot file from `world/plots`, which includes its pending ticks.
A circuit is a [RIL](RIL.md) file (`.ril`).

A setup is a backend followed by its flags, written as after `/redpiler compile`: `-b "direct -o -i"`, `-b vanilla`.

- `reference` exists only in rilc, beside the order oracle. It runs the graph without passes of its own, directly on the [node behaviour](Circuits.md#node-behaviour).
- `vanilla` exists in rilc and in the [library](Library.md), not in the server. It runs worlds only.
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

## Fuzzing

`--seed <s>` sets the seed of the first case, 1 by default, and `--cases <n>` the number of cases, 1000 by default.
A failing case prints the command that runs it alone and the failure: for a difference, what differs and the last input.
A failing circuit is also written to `.scratch/fuzz` as `<seed>.ril` and `<seed>.toml`, for `rilc diff`.
| Command | What runs | What it checks |
| --- | --- | --- |
| `fuzz engine` | `direct` on random circuits, or on the worlds and RIL circuits it is given. | After every input and every tick, the state of every node and the pending ticks are legal, and the reported changes bring the observed nodes to the state of the snapshot. |
| `fuzz passes` | Passes on a small random circuit, or on the worlds and RIL circuits it is given: each pass alone, and each pass of `direct --optimize --io-only` after its predecessors, or the list given with `--passes`. | What the observed nodes of the result show on `reference` is legal for the circuit without the passes. |
| `fuzz worlds` | Random worlds on the interpreter of `mchprs_redstone` and on `reference`, `direct -o` and `direct -o -i`. | After every input and every tick, what the interpreter shows and its pending ticks are legal for the world's circuit. A setup may differ from the interpreter only in the components it shows, and only if its run is legal too. |

`fuzz worlds` also stops `reference` after a random tick and lets `vanilla` run on from what it wrote back.

### The Order Oracle

All three fuzzers judge by the [tick order of `direct`](backends/Direct.md#tick-order). *Legal* means allowed by it.
The order oracle in [`crates/rilc/src/oracle`](../crates/rilc/src/oracle/mod.rs) decides it: it runs the `reference` engine in every order that the tick order leaves unspecified.
What was seen is legal if one of these runs shows it.

- Ticks of one priority run in the order they were scheduled.
- The ticks that one change schedules with one delay and priority, by updating the nodes it powers, run in every order. The order of those updates is unspecified.
- Pending ticks are seen in the order they run in. That order must be one the oracle follows, and the oracle then follows no other.
- Whether a note block played is seen after every tick, as one answer for all the times it became powered since the tick before.
- A fuzzer skips a case in which the orders of one priority of one tick reach more states than the budget of the oracle.

The oracle does not try every order in which a node updates the nodes it powers. These suffice:

- Updates of different targets differ only in the order of the ticks they schedule, and the oracle runs the ticks that one change schedules in every order.
- Links of one kind into one target move that input one way, so one order of them stands for all.
- A comparator fed through both inputs schedules a tick in some order only if it does so rear first or side first, and schedules none only if it schedules none when updated once for both. So those three cover every order.
- A repeater fed through both inputs, updated once, does as side first if its side input turns on and as rear first otherwise. It needs no third order.

## Tests

`rilc test crates/redpiler/ril_tests` runs the tests of the passes. `--update` replaces the expected circuit of each failing test with its result.
