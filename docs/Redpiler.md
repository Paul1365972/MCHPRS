# Redpiler

Redpiler compiles the redstone of a plot and runs it on a backend.
[Semantics](Semantics.md) defines how a compiled plot differs from vanilla.

## How It Works

Every backend that the server offers transforms the plot into a directed weighted graph.
The nodes are the redstone components, and the weight of a link is the signal strength lost in the redstone wire between two of them.

In a plot without Redpiler, a component looks for all its sources of input whenever it is updated.
Redpiler looks once, at compile time, and keeps the sources as the links of the graph.

A backend can run passes that rewrite the graph before it runs it.
[Circuits](Circuits.md) describes the graph, the passes and the behaviour of each component.

## Backends

| Backend | Description |
| --- | --- |
| [`direct`](backends/Direct.md) | Runs the graph node by node, one tick after another. It is built for speed. |

[`vanilla`](backends/Vanilla.md) runs a copy of the plot with the same code as a plot without Redpiler.
The page of a backend states its tick order, its passes, its flags and its limits. [Semantics](Semantics.md#backends) compares what the backends compute.

## Flags

Every backend has its own flags. Its page lists them, and so does `/help redpiler compile <backend>`.

Flags go after the backend. Short flags can be combined: `/redpiler compile direct -io`.

The [config](../README.md#redpiler) sets default flags per backend.
Flags given to the command are added to the defaults.
`--<flag>=false` or `-<short>=false` turns a default off: `/redpiler compile -o=false`.
