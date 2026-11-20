# phase

The `phase` backend runs the [graph](../Circuits.md) of a plot with all ticks of one priority at once.
It is the reference for this tick order, which [`fpga`](FPGA.md) runs as a digital circuit.
A server built with the `fpga` feature offers it, so that a player can tell whether a result comes from this tick order or from the board.
[rilc](../rilc.md) and the [library](../Library.md) offer it too.

[Semantics](../Semantics.md#compiled-plots) lists how every compiled plot differs from vanilla. This page covers what is specific to `phase`.

## Tick Order

Ticks run by redstone tick, then by priority.
All ticks due with one priority run at once: each reads the circuit as it was before any of them ran, and their changes commit together.
Then each component whose inputs changed is updated once, with its final inputs, before the next priority runs.
The result does not depend on the order of nodes, links or scheduling.

A build without a [race](../Semantics.md#races) behaves as under [`direct`](Direct.md).
A build with a race has one result under `phase`, the same in every compile, which can differ from `direct` and from vanilla even if the build is not locational.

For example, a tick turns on the side input of a subtract comparator, and the comparator's own tick to turn on is due in the same redstone tick with the same priority.
Under `phase` the comparator pulses for one redstone tick.
Under `direct` it pulses only if its tick was scheduled first.

## Passes

Without `--optimize`, `phase` runs none.
With it, it runs `fold`, `unify-constants`, `prune-orphans` and its own [`coalesce`](#coalesce), in this order.
It leaves out `sort-links`, which only sets the order of updates.
`phase` has `--optimize` because `fpga` runs the same passes, so comparing `phase` with `phase -o` checks them without a board.

### Coalesce

Under `phase`, nodes in the same state that are powered the same way always change together.
So its `coalesce` merges nodes that agree in kind, port role, state, pending tick, whether they are observed, and incoming links, without the conditions on order of [that of `direct`](Direct.md#coalesce).
It also merges nodes with a pending tick, nodes that are their own input and identical loops. It does not merge inputs.
Comparators whose only changing input is one source of two strengths also merge when both strengths give them the same outputs.

The pass is only correct in the tick order of `phase`, so `--passes` cannot name it.
[`rilc fuzz agreement`](../rilc.md#fuzzing) checks it with `-b phase -b "phase -o"`.

## Flags

`phase` takes the [flags of `direct`](Direct.md#flags).

## Limits

`phase` cannot halt on [breakpoints](../Redpiler.md#breakpoints).
