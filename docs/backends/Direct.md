# direct

The `direct` backend runs the [graph](../Circuits.md) of a plot node by node, one tick after another.
It is built for speed and is the backend that [Redpiler](../Redpiler.md) uses unless another is configured.

[Semantics](../Semantics.md#compiled-plots) lists how every compiled plot differs from vanilla. This page covers what is specific to `direct`.

## Tick Order

A *scheduled tick* belongs to one component, is due in one redstone tick and has a priority.
When a scheduled tick runs, its component may change. If it changes, every component it powers is updated, and an update may schedule a new tick.
A *step* is one scheduled tick running, together with the updates that follow from it. Driving an input is a step too.

### Rules

1. Steps never overlap. One step finishes before the next one starts.
2. Scheduled ticks run sorted by redstone tick, then by priority, then by the order in which they were scheduled.

### Unspecified

Two things inside a step are left open:

1. The order in which the powered components are updated.
2. Whether a component is updated once or twice when the same change reaches both its rear and its side input.

Everything else is fixed.

### Consequences

- The ticks scheduled by one step stay together. They run before the ticks scheduled by any later step.
- A graph can have several valid tick orders. They lead to different results only if the build is [locational](../Semantics.md#ticks-and-updates).
- Which valid tick order runs can change from one compile to the next, so a locational build is not guaranteed to behave the same after a recompile.

A build that is not locational behaves as in vanilla, except for some builds with [lamps](../Semantics.md#lamps).

## Passes

Without `--optimize`, `direct` runs only `sort-links`.
With it, it runs these [passes](../Circuits.md#passes) in order:

1. `fold`
2. `unify-constants`
3. `prune-orphans`
4. `sort-links`
5. `coalesce`

### Coalesce

A merged node ticks once where its nodes tick one after another.
Each of them updates the nodes it powers, so these are scheduled node by node, and the merged node has to schedule them in the same order from one list of links.

`coalesce` puts the nodes of a merged node in an order.
Nodes that could change are merged only if both of the following hold:

- No two of them power the same input of a node that ticks. Each of the two would update it, and the merged node updates it once.
- The links of the merged node have an order in which no input is powered by an earlier node than an input before it in the list.

Two inputs of different nodes are exempt from the second condition if the order of the two nodes never matters:

- The two nodes cannot be scheduled with the same delay and priority, so they never share a queue.
- The two nodes are independent: neither powers the other, they power no common node, and every two nodes that their ticks can schedule into one queue are independent too.

`coalesce` does not find every merge that these conditions allow.

## Flags

| Flag | Short | Description |
| --- | --- | --- |
| `--wire-dot-out` | `-d` | Treat redstone wire without connections (dots) as outputs. |
| `--wire-cross-out` | `-c` | Treat redstone wire connected on all four sides (crosses) as outputs. |
| `--illegal-states-out` | `-l` | Treat redstone wire connected on exactly one side as outputs. |

Inputs are levers, buttons and pressure plates.
Outputs are redstone lamps, iron trapdoors, note blocks and the wires that the last three flags select.
[Shown Blocks](../Semantics.md#shown-blocks) states what `--optimize` and `--io-only` change about the blocks shown.
