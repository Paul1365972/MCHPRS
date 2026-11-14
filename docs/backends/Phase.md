# phase

The `phase` backend runs the [graph](../Circuits.md) of a plot with all ticks of one priority at once.

It is the reference for this tick order.
It is not built for speed.
A started simulation starts paused.
An input while paused runs no tick.
A counted run or a settle leaves the simulation paused.

[Semantics](../Semantics.md#compiled-plots) lists how every compiled plot differs from vanilla. This page covers what is specific to `phase`.

## Tick Order

Ticks run by redstone tick, then by priority.
All ticks due with one priority run at once.
Each reads the circuit as it was before any of them ran.
All changes commit together.
Then every component with a changed input is updated once with its final inputs.
This happens before the next priority runs.
The result does not depend on node order, link order or scheduling order.

A build without a [race](../Semantics.md#races) behaves as under [`direct`](Direct.md).
A build with a race has one result under `phase`, the same in every compile.
That result can differ from `direct` and from vanilla, even if the build is not locational.

For example, a tick turns on the side input of a subtract comparator, and the comparator's own tick to turn on is due in the same redstone tick with the same priority.
Under `phase` the comparator pulses for one redstone tick.

## Passes

Without `--optimize`, `phase` runs none.
With it, it runs these [passes](../Circuits.md#passes) in order:

1. `fold`
2. `unify-constants`
3. `prune-orphans`
4. its own [`coalesce`](#coalesce)

`sort-links` is not among them: it only sets the order of updates, which `phase` does not depend on.

### Coalesce

Under `phase`, nodes that are in the same state and are powered the same way always change together.
So `phase` has its own `coalesce` in place of [that of `direct`](Direct.md#coalesce), without the conditions on order.

It merges nodes that agree in kind, port role, state, pending tick, whether they are observed, and incoming links.
This includes nodes with a pending tick, nodes that are their own input and the nodes of identical loops.
Inputs are not merged.

The pass is only correct in the tick order of `phase`. `--passes` cannot name it, and `rilc fuzz passes` does not check it.

## Flags

| Flag | Short | Description |
| --- | --- | --- |
| `--optimize` | `-o` | Run the passes. Redstone wire is removed from the graph unless it is an output or a probe reads it. |
| `--io-only` | `-i` | Send block changes only for inputs, outputs and the components probes read. |
| `--wire-dot-out` | `-d` | Treat redstone wire without connections (dots) as outputs. |
| `--wire-cross-out` | `-c` | Treat redstone wire connected on all four sides (crosses) as outputs. |
| `--illegal-states-out` | `-l` | Treat redstone wire connected on exactly one side as outputs. |

Inputs are levers, buttons and pressure plates.
Outputs are redstone lamps, iron trapdoors, note blocks and the wires that the last three flags select.
[Shown Blocks](../Semantics.md#shown-blocks) states what `--optimize` and `--io-only` change about the blocks shown.

## Limits

`phase` cannot halt on [breakpoints](../Redpiler.md#breakpoints).
