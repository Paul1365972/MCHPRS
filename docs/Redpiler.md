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

A backend owns its threads and decides when its ticks run.
[`vanilla`](backends/Vanilla.md) runs a copy of the plot with the same code as a plot without Redpiler.
The page of a backend states its tick order, its passes, its flags and its limits. [Semantics](Semantics.md#backends) compares what the backends compute.

## Commands

| Command | Alias | Description |
| --- | --- | --- |
| `/redpiler compile [<backend>]` | `/rp c` | Compile the plot and run it on the given backend. Without a backend, the [configured default](../README.md#redpiler) is used. If Redpiler is running or compiling, it is reset first. |
| `/redpiler reset` | `/rp r` | Stop Redpiler or cancel a running compile. Write the state of every component back into the plot, then update every block of the plot. |

`/redpiler reset --restore` (`-r`) returns every block and pending tick to its state before compiling instead.

While Redpiler compiles or runs:

- Blocks cannot be placed, broken or changed by hand, such as the delay of a repeater, and commands that change blocks fail.
- Levers, buttons and pressure plates work once it runs, not while it compiles.
- `/rtps` and `/radvance` apply to the compiled plot. See [Commands](Commands.md#general).

`/redpiler compile` saves the plot before compiling.
The plot is not saved again until Redpiler stops.
Redpiler stops on `/redpiler reset`, on a new `/redpiler compile`, when the plot unloads and when the server stops.
If a backend fails while running, the plot is restored to its state before compiling.
A backend that cannot read its state back restores the plot on `/redpiler reset` too, and says so before.
A plot that Redpiler [refuses](Semantics.md#refused-plots) stays as it was.

## Flags

Every backend has its own flags. Its page lists them, and so does `/help redpiler compile <backend>`.

Flags go after the backend. Short flags can be combined: `/redpiler compile direct -io`.

The [config](../README.md#redpiler) sets default flags per backend.
Flags given to the command are added to the defaults.
`--<flag>=false` or `-<short>=false` turns a default off: `/redpiler compile -o=false`.

## Probes

A probe reads up to 64 components as one number, one bit per component.
A bit is 1 while the signal strength of its component is above 0.
The sidebar shows the value of every probe in hexadecimal.
A probe shows `broken` while one of its positions holds no component.

| Command | Description |
| --- | --- |
| `/probe add <name> [<mask>] [<direction>]` | Add a probe on the components in the WorldEdit selection that match the mask. Without a mask, every component in the selection is read. |
| `/probe remove <name>` | Remove a probe. |
| `/probe list` | Show the value and breakpoint of every probe. |
| `/probe clear` | Remove every probe. |

The bits of a probe are ordered along `<direction>`, lowest bit first.
The default direction is `me`, the direction the player faces.
Components at the same position along the direction are ordered by y, then x, then z.

Probes are not saved with the plot.

## Breakpoints

A breakpoint halts the plot when the value of a probe meets a condition.
A probe has at most one breakpoint.

| Command | Description |
| --- | --- |
| `/probe break <name> == <value>` | Halt when the value becomes `<value>`. |
| `/probe break <name> != <value>` | Halt when the value changes from `<value>` to another value. |
| `/probe break <name> changes` | Halt when the value changes. |
| `/probe unbreak <name>` | Remove the breakpoint of the probe. |

`<value>` is decimal, hexadecimal or binary: `47`, `0x2F` or `0b101111`.

The condition is checked at the end of every redstone tick against the value at the end of the previous tick.
`== 5` set while the probe reads 5 halts only after the value has left 5 and returned.
A value that changes and changes back within one tick does not halt the plot.

A halt ends a running `/radvance` and pauses the plot, as `/rtps 0` does.
`/rtps`, `/radvance` and `/redpiler compile` end a halt, and so does `/redpiler reset` while Redpiler runs.
A backend that cannot halt refuses `/probe break`.
