# Library

The redstone backends of MCHPRS run without the server.
A program loads a world, compiles it with a backend into a simulated world, drives its inputs, runs ticks and reads its blocks.
[Writing a Backend](Writing_a_backend.md) covers adding one.

## Example

[`crates/backends/examples/schematic.rs`](../crates/backends/examples/schematic.rs) does this for [`lamp.schem`](../crates/backends/examples/lamp.schem): a lever, a repeater with a delay of 2 and a redstone lamp in a row.
It settles the world, compiles it for `vanilla` and for `direct -o`, flips the lever and prints whether the lamp is lit after each of two ticks. It then stops the simulation, applies its write-back to the shown blocks and continues in the interpreter.

```sh
cargo run -p mchprs_backends --example schematic
```

```text
vanilla, tick 1: lamp lit: false
vanilla, tick 2: lamp lit: true
direct -o, tick 1: lamp lit: false
direct -o, tick 2: lamp lit: true
```

## Dependencies

The example uses these crates of this repository, which must come from the same revision:

```toml
[dependencies]
mchprs_backends = { git = "https://github.com/Paul1365972/MCHPRS" }
mchprs_simulation = { git = "https://github.com/Paul1365972/MCHPRS" }
mchprs_blocks = { git = "https://github.com/Paul1365972/MCHPRS" }
mchprs_redstone = { git = "https://github.com/Paul1365972/MCHPRS" }
mchprs_schematic = { git = "https://github.com/Paul1365972/MCHPRS" }
```

| Crate | Provides |
| --- | --- |
| `mchprs_backends` | `backends()`, the backends the server offers, `VanillaBackend`, and `Setup`, a backend with its [flags](Redpiler.md#flags) |
| `mchprs_simulation` | `Source`, the world to compile, and `SimulatedWorld`, the compiled world, stepped by the program |
| `mchprs_blocks` | `Block`, `BlockPos`, `BlockVolume` and `BlockView`, which reads blocks |
| `mchprs_redstone` | `Input`, `settle`, and `Interpreter`, a world that runs block by block |
| `mchprs_schematic` | `read`, which reads Sponge schematics |

`mchprs_backends` has a feature per backend, `direct` and `vanilla`, both on by default.
`backends()` does not list `vanilla`. `Setup::new(&VanillaBackend, FlagValues::default())` sets it up.
With `default-features = false, features = ["vanilla"]`, `backends()` returns none and the Redpiler crates are not compiled.

## Rules

- **Settle first.** A world with a [stale](Semantics.md#baseline-requirements) component is [not compiled](Semantics.md#refused-plots) into a graph. `settle` updates every block of a region and runs the world until no tick is pending; it returns `None` if ticks are still pending after the given number of ticks. A world with pending ticks passes them in `Source::ticks`.
- **Not every block is current.** [Shown Blocks](Semantics.md#shown-blocks) states which blocks a backend can leave as they were at compile time. `SimulatedWorld::is_current(pos)` tells whether the block at `pos` is current. The positions in `Source::observed` and `Source::declared` stay current.
- **Inputs change between ticks.** `SimulatedWorld::drive` applies an `Input` to a lever, button or pressure plate; [Inputs](Circuits.md#inputs) lists which `Input` each takes. A pressure plate is untouched after compiling until it is driven with `Input::Touch(true)`.
- **Ticks run only inside a call.** `SimulatedWorld::run` runs a number of ticks and returns when they have run. `SimulatedWorld::settle` runs until no tick is pending, at most the given number of ticks. `SimulatedWorld::drive` returns when the input has landed. After each call the world shows what the simulation observes.
- **The write-back refreshes the region.** `SimulatedWorld::stop` returns the shown blocks and a `WriteBack`: the component changes not yet shown, the pending ticks and the compiled region. `WriteBack::apply` writes them and updates every block of the region. The update brings omitted blocks, such as plain wire with `direct -o`, up to date. A backend that did not read every component back returns `Readback::Incomplete` in its place.
- **Reading ends nothing.** `SimulatedWorld::read` returns the same `WriteBack` and the simulation goes on.
- **Unknown blocks become air.** `mchprs_schematic::read` replaces the block states it does not know with air and counts them in `Schematic::unknown_blocks`.

## WebAssembly

`mchprs_redpiler`, `mchprs_backends` and `mchprs_simulation` build for `wasm32-unknown-unknown`.
A `Simulation` is stepped on the caller's thread, so a `SimulatedWorld` runs there.
`Simulation::start` needs a thread of its own, which that target lacks when it runs. There `start` returns an error.
