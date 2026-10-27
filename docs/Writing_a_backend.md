# Writing a Backend

A backend turns a region of blocks into a simulation that the server, rilc and the [library](Library.md) run.

| Crate | Content |
| --- | --- |
| `mchprs_simulation` | What every backend shares: the `Backend`, `Simulation` and `Running` traits, `Space` and `Blocks`, the components of a world, the `SimulatedWorld`, breakpoints, and `stepper::start`, which runs a `Simulation` on a thread |
| `mchprs_redpiler` | The `GraphBackend` trait, `Nodes`, the graph, the passes, the node behaviour and the block map |
| `mchprs_redpiler_direct` | The `direct` backend |
| `mchprs_redpiler_phase` | The `phase` backend |
| `mchprs_redpiler_fpga` | The `fpga` backend |
| `mchprs_backends` | `Setup`, the list of backends the server offers, and `vanilla` |
| `mchprs_redstone` | The block-by-block simulation that runs a plot without Redpiler |

Every backend implements `Backend`. `GraphBackend: Backend` adds graph compilation:

- A `Space` names what a simulation is addressed by and what it reports. `Blocks` addresses a plot by block position. `Nodes` addresses a circuit by node.
- `Simulation<S>` is stepped by its caller: it runs ticks only inside `run` and `settle`. `read` returns its whole state without changing it. `stop` ends it and returns what was not shown yet with the state it reached.
- `Running<S>` runs by itself. It owns its threads and its timing, takes commands with `send` and answers with updates. `Simulation::start` turns a stepped simulation into a running one, paused, with its state, pending ticks, watches and tick count. The doc comments of both traits and their types state the contract.
- `Backend::compile` turns a region of blocks into a `(Box<dyn Simulation<Blocks>>, Layout)`. `vanilla` steps the interpreter over a copy of the region. It builds no graph.
- `GraphBackend::build` turns a circuit into a `Box<dyn Simulation<Nodes>>`. `direct`, `phase`, `fpga` and rilc's `reference` are graph backends. `graph_simulation::compile` extracts the graph, runs the passes, builds the circuit's simulation and addresses it by block position through the block map.
- `stepper::start` gives a `Simulation` a thread that runs its ticks. `direct`, `phase`, `vanilla` and `reference` start that way. `fpga` has its own thread, which sets the board's run mode and reads the board back.
- The simulation that a graph backend builds checks each input with the circuit's `InputKinds`.

`rilc run` measures how fast a backend runs through `Running`, the way the server runs it. Speed is never measured on a stepped simulation.

The server offers the backends that `GRAPH_BACKENDS` in [`crates/backends/src/lib.rs`](../crates/backends/src/lib.rs) lists, available with the `graph` feature, which each graph backend's feature enables.
`backends()` returns them as `Backend`s.
rilc uses this list and adds `vanilla`, and `reference` for `--passes` and the circuit commands.
Adding a backend means adding it to this list and declaring its feature and optional dependency in `mchprs_backends`.
With no features, `Setup` and `ParseError` remain available and `backends()` returns none.

A backend gets a page under [`docs/backends`](backends) with its tick order, passes, flags and limits, and a row in the tables of [Redpiler](Redpiler.md#backends) and [Semantics](Semantics.md#backends).
