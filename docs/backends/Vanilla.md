# vanilla

The `vanilla` backend runs a copy of the plot's blocks with the code that runs a plot without Redpiler.
It builds no graph, runs no passes and takes no flags.
The server does not offer it. [rilc](../rilc.md) and the [library](../Library.md) do.

[Semantics](../Semantics.md) defines the other backends by how they differ from it.

## Tick Order

The same as a plot without Redpiler, which is the [baseline](../Semantics.md#baseline-requirements).

Ticks run by redstone tick, then by priority, then in the order they were scheduled.
A changed block updates its neighbours at once, in vanilla's order of positions.
So a [locational](../Semantics.md#ticks-and-updates) build behaves as in vanilla at the same position and facing.

## Limits

- It is as slow as a plot without Redpiler.
- A [declared input](../Circuits.md#ports-and-observed-nodes) has to be a lever, a button or a pressure plate. `vanilla` cannot cut another component off from the blocks that power it.
