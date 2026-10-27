# phase

The `phase` backend runs the [graph](../Circuits.md) of a plot with all ticks of one priority at once.
It is the reference for this tick order.
The server does not offer it, [rilc](../rilc.md) and the [library](../Library.md) do.

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
With it, it runs `fold`, `unify-constants` and `prune-orphans`, in this order.
It leaves out `sort-links`, which only sets the order of updates.

## Flags

`phase` takes the [flags of `direct`](Direct.md#flags).

## Limits

`phase` cannot halt on [breakpoints](../Redpiler.md#breakpoints).
