# Semantics

MCHPRS implements the redstone mechanics of Minecraft 1.20.4.
This page states where it differs from vanilla, in three steps. Each step keeps the differences of the one before.

1. [The baseline](#baseline-requirements): a plot without Redpiler, and the `vanilla` backend.
2. [Compiled plots](#compiled-plots): every backend that runs a graph.
3. [Backends](#backends): the tick order of each backend.

## Baseline Requirements

A plot behaves identically to vanilla under the following assumptions:

1. It contains only supported components: levers, buttons, pressure plates, redstone torches, repeaters, comparators, redstone wire, redstone lamps, iron trapdoors, note blocks, constant sources such as redstone blocks, and blocks that comparators read, such as barrels. [Nodes](Circuits.md#nodes) lists every block.
2. The world is stable: only inputs (levers, buttons and pressure plates) and ticks change it. No block is placed, broken or changed by hand, such as the delay of a repeater.
3. No component is stale. A component is stale if its inputs would change it and no tick is scheduled to do so. Simulation never produces a stale component, edits can. `//update -p` updates every block of the plot and schedules the missing ticks.

## Omitted Features

| Feature | MCHPRS |
| --- | --- |
| Torch burnout | Torches never burn out. |
| Observers | Exist, but never pulse. |
| Pistons, droppers, dispensers, daylight detectors, weighted pressure plates, wooden trapdoors | Do not exist. |
| Hoppers | Hold items for comparators. They neither move items nor lock. |
| Iron trapdoors | No redstone component can be placed on them. |
| Game ticks | Time advances in redstone ticks of two game ticks. Inputs take effect between redstone ticks. |
| Scheduled tick limit | Every due tick runs. Vanilla runs at most 65,536 per game tick. |

## Compiled Plots

[Redpiler](Redpiler.md) runs a plot on a backend.
The [`vanilla`](backends/Vanilla.md) backend has the semantics above.

Every other backend compiles the plot into a [graph](Circuits.md) of components and links, and runs it without the positions of their blocks.
A compiled plot differs from the semantics above as follows.

### Ticks and Updates

Time advances in redstone ticks. A component changes its output only when one of its scheduled ticks runs or when it is driven as an input.
Ticks run by redstone tick, then by priority.
When a component changes, everything it powers is updated.

In vanilla, two things about these updates follow from the positions of the blocks:

- Their order.
- Whether a component that one change powers through two inputs is updated once or twice.

A changed block updates its neighbours in a fixed order of directions, and redstone wire updates its neighbours in the iteration order of a Java `HashSet` of their positions.

A build is *locational* if its result depends on either. In vanilla its result then depends on where it is built and which way it faces.
A graph has no positions, so no compiled plot promises a locational build the result it has in vanilla.

### Races

A *race* is one of the following:

- A scheduled tick changes an input of a component whose own tick is due in the same redstone tick with the same priority.
- Two ticks due in the same redstone tick with the same priority change inputs of one component.
- One change reaches both the rear and the side input of one component. Most builds that compute contain this; a subtract comparator with equal inputs then does nothing or schedules a tick that changes nothing.

Only a build with a race can be locational.
A build without a race behaves as in vanilla on every backend, except for some builds with [lamps](#lamps).
[Backends](#backends) states what each backend does with a race.

### Lamps

A lamp can be lit in a compiled plot while it is unlit in vanilla, or the other way round, for up to two redstone ticks.
This happens only if both of the following hold:

- In the redstone tick in which the lamp's off tick is due, and before that tick runs, a component within two blocks updates the lamp without powering it: redstone wire, a torch, a button, a pressure plate, a note block, or a repeater or comparator that faces a block beside the lamp. Vanilla then schedules a second off tick. A graph updates a lamp only through its inputs.
- After that update the lamp's input turns on, and off again in one of the next two redstone ticks, before the second off tick runs.

If the lamp's input turns on in the same redstone tick as that update, the result in vanilla can depend on the build's position and facing.

### Refused Plots

A plot is not compiled into a graph in the following cases:

- A component is stale.
- A container or comparator has a signal strength above 15.

### Shown Blocks

A backend does not have to show the state of every block. This changes what the blocks show, not what the build computes.
`direct` has two flags for it:

- `--optimize`: redstone wire keeps the power it had at compile time, unless it is an output or read by a probe.
- `--io-only`: only inputs, outputs and components read by probes show their state.

A reset shows every block again.

## Backends

The backends differ in how the ticks of one priority run and in when a change updates what it powers.
For every backend but `vanilla`, these come on top of the differences of [Compiled Plots](#compiled-plots).

| Backend | Ticks due with one priority | What a change powers is updated | A build with a race |
| --- | --- | --- | --- |
| [`vanilla`](backends/Vanilla.md) | Run in the order they were scheduled. | At once, in vanilla's order of positions. | Behaves as in vanilla. |
| [`direct`](backends/Direct.md) | Run in the order they were scheduled. | Before the next tick runs, in an unspecified order. | Behaves as in vanilla, unless it is locational. Then its result is unspecified. |
