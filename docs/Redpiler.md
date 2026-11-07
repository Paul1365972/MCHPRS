# Redpiler

At a high level, Redpiler transforms the redstone in a Minecraft world into a directed weighted graph. The weights represent the distance taken by a redstone signal when travelling through a wire. Since Redpiler was inspired by the design of modern compilers such as LLVM, the graph then goes through several passes which are run depending on how Redpiler was configured. Every pass is an optional optimization.

The final graph becomes a `Circuit`, which an engine runs. A `BlockMap` maps the circuit's nodes to their blocks: it turns block positions into inputs and the engine's changes back into blocks.

A `Circuit` serializes with serde as its nodes and its links in update order, for programs that want the graph Redpiler compiles. Deserializing one checks it like `Circuit::new` does.

# Extraction

Extraction turns the plot into the nodes and links of the graph.

## Nodes

Extraction iterates through all the blocks in the plot and tries to identify them as Redstone components. If a block is a Repeater, Comparator, Torch, Button, Lamp, Lever, Pressure Plate, Iron Trapdoor or Note Block with air above it, a new node is created in the graph with the appropriate node type containing the necessary state information. Redstone Wires are added too, except that with `--optimize` only wires that are ports are added.

Blocks that have a comparator override such as Barrels, Furnaces, Hoppers, Cauldron, Composters, and Cake are also added into the graph as constant nodes, and so are Redstone Blocks.

Redpiler runs signal strengths up to 15, while containers can hold up to 1779 and comparators pass on what they read. Extraction refuses a circuit with a container or comparator holding more than 15, or a comparator reading more than 15 through a block.

Some nodes are ports: inputs set from outside the circuit, and outputs watched from outside it. Levers, buttons and pressure plates are inputs; lamps, trapdoors and note blocks are outputs, and the `--wire-*-out` flags make matching wires outputs too. Observed nodes report their changes while the circuit runs: every node, or with `--io-only` only the ports.

The plot's pending ticks become the pending ticks of their nodes. Ticks due together with the same priority keep their order from the plot's tick queue.

## Links

Now that the graph has been populated with nodes, Redpiler can start finding the connections between Redstone components.

This is the most complex part of the compile, but it is also one of the most important aspects of Redpiler. One of the major reasons vanilla Minecraft is so slow at processing redstone is because of  the fact that when a component is updated, which can happen several times in a tick, it has to look for all sources of input. This can take a relatively very long time, and Redpiler solves this issue by only looking once and saving this information in the form of the links in the graph.

To start, extraction iterates through all nodes in the graph. Different types of nodes need to be handled differently. For example, Torches need to look for links on the block it is placed on, but Comparators and Repeaters need to look for links in the direction it is facing as well as links from the side.

When the input block of a node is searched, the block is either a component that can provide Redstone power on its own, or a Redstone Wire. If it can provide power, then it can directly create a link to that component. The corresponding node in the graph is looked up based on the position of the component, and a link to the node is created with a weight of 0. If the block is a Redstone Wire, then a breadth-first search is run to look for components that provide power to the Wire. The distance of the path taken from the starting wire to the input components are recorded as the weight of the links. Then, input components are looked up in the graph, and links are created.
The search stops after 15 wires, so every link weighs less than 15.

Each node's outgoing links are ordered by weight, then by target type, then by target index (with `--reorder`, by target type first), which is the order the engine updates them in.

# The Pass System

Passes receive mutable access to the compile graph. Each backend chooses which passes run, in order; without `--optimize`, none do.

## The `DedupLinks` Pass

Sometimes, the breadth-first search done by extraction can result in two different paths to the same node. While this would not cause any problems during execution, it is still inefficent. This optimization pass removes duplicate links to the same node, only keeping the link with the lowest weight. For example, if two nodes are connected with two links of weights 13 and 14, the link with weight 14 is removed.

## The `ConstantFold` Pass

While nodes that are never updated in theory have no affect on the number of instructions that are run at runtime, therefore the time taken to perform a tick at runtime, keeping the size of the graph small helps to avoid cache misses that to end up taking time at runtime. This optimization pass reduces the size of the final graph by recognizing situations where a node only has constant inputs and tranforming that node into a constant node, breaking the links to the other constant nodes.
Nodes with pending ticks are not folded. A folded node keeps its current output strength and, for a repeater, its lock, because a node whose inputs are all constant is never updated again.

## The `UnreachableOutput` Pass

A link only carries a signal if its source can output a strength above the link's weight.
The `ss_sets` analysis provides each node's maximum output strength, including its current output and any pending tick.
Every outgoing link whose weight is at least that maximum is removed.
For example, a comparator in subtract mode with a constant side input never outputs more than its maximum default input minus that constant, so links with a higher weight are removed.

## The `ss_sets` Analysis

This analysis approximates output strengths using the compile graph's node states and inputs.
Every node starts with its current output strength.
Then the sets of its inputs are decayed by the link weights and combined, and the node type determines which outputs these inputs can produce.
Evaluation results are unioned into the existing set.
Nodes are visited in index order, and a node whose set grew marks the nodes it feeds for the next sweep, until a sweep changes nothing.
Constants retain their current strength; buttons, levers and pressure plates admit both 0 and 15.

Pending repeaters include both 0 and 15: a queued tick can turn an unlocked repeater on after its input disappears. Torches and comparators instead reevaluate their inputs when ticked.
A repeater whose side input is never off keeps its output, locked or not: every update locks it before evaluating its power.

Since each set has only 16 possible strengths and sets only grow, the sweeps terminate.
Cycles can retain bounded output sets, for example a comparator loop fed by a bounded signal.
The sets do not track correlations between inputs, so they can include combinations that cannot occur together at runtime.

## The `Coalesce` Pass

There are often times when a wire powers many different components in the same way. For example, it is common for vertical multi-bit latches to be controlled by a slab tower that powers several repeaters that lock other repeaters. This is very inefficient because these repeaters will always have the exact same value, but they are still updated and ticked independently. To avoid this logic duplication, this optimization pass merges duplicate nodes into one, removing duplicate nodes from the graph and adjusting links to point to the new node.
Nodes merge when they have the same type, port role, initial state and input links. Inputs and nodes with a pending tick never merge.
The merged node keeps the blocks of all merged nodes and writes each of them.
Input links match by source, link type and weight, counting only the strongest link per source and link type.
The weight doesn't count when a binary source (everything except comparators, wires and constants) feeds a binary reader (everything except comparators and wires), as any link weighs less than 15.
After a merge, the nodes fed by the merged node are checked again.

The merged node updates the targets of one merged node after another, as if they ticked right after each other (see [Tick Ordering](#tick-ordering)). This can change the order of updates within a tick, so the pass only runs with `--reorder`.
Nodes that read their own output, or whose inputs update each other, never merge: an input could change between their ticks.

## The `PruneOrphans` Pass

This pass removes every node that isn't observed and doesn't feed an observed node, directly or through other nodes.
Nodes with pending ticks and the nodes feeding them stay.
It only removes nodes with `--io-only`; otherwise every node is observed.

# Running Circuits

A backend builds an engine for a compiled circuit. The engine runs the circuit from its own data, as many ticks as it is asked to, and reports the observed nodes whose state changed. It never reads a clock. A `Simulation` pairs it with a `BlockMap` on the caller's thread, which is how `rilc`, the tests and other programs run circuits; `BackendSetup::simulate` builds one from blocks in a single call, without threads.

The server runs a circuit as a session, which owns how its ticks happen; the plot thread never ticks. A backend can host its circuits itself, otherwise its engine runs on a worker thread that paces it by the plot's tick rate and runs `/radvance` on top. The worker sends its changes as frames: one each time the plot asks for the next, and one right away after an input or when an advance ends. The plot keeps the `BlockMap` and writes the frames into the world.

The plot reads a session in two ways. Polling, at every plot update, never waits, and a backend may hand over changes from different ticks at once. A flush waits for a tick boundary and carries every node's state; commands that read the world flush first, and stopping waits for a boundary too. If a session doesn't reach one within 10 seconds, a flush gives up and the world lags behind, while a stop abandons the session and restores the plot as it was saved before compiling.

The world can't be edited while Redpiler runs, so the engine's state only reaches the world this way. While running, the world shows the nodes the circuit observes, which are only the ports with `--io-only`, and the other nodes as compiling or the last flush left them; players joining see that. Stopping writes every node's state and the pending ticks back, and `/redpiler reset -r` returns every component to its compiled state first.

Direct runs a circuit the same way every time, while a backend that runs parts of a circuit in parallel may leave the order of some updates open. Backends differ by design: like `--reorder`, each is an optimization level, and `rilc diff` shows where two of them part.

# Backends

A backend runs a circuit. It may implement redstone execution in any way, whether that is by just-in-time compiling redstone or by interpreting the graph.

[`rilc diff`](#diff) plays a scenario on two backends tick by tick and explains the first difference, which checks an optimized circuit against the plain one, or one backend against another.

## How Redstone Works

Implementing a Redstone executor is not as trivial as it may initially seem because Redstone has very specific timing rules.

There is a difference between a Redstone component being updated and being ticked. When a redstone component is updated, it might calculate what it's state should be, and if that state is different from what its current state is, it may schedule a block tick for this component at some delay and priority. When the component is ticked, the state may change. There are 4 tick priorities which Redpiler names `Normal`, `High`, `Higher`, and `Highest`. Others may have different names for these priorities, or use number instead completely avoiding names, but it is all the same.

For example, when a Torch that is off (being powered) is updated, and it finds that it is no longer being powered and there is no tick already scheduled at this node, it will schedule a tick with delay 1 and priority `Normal`. When the time has finally come for the node to be ticked, the torch checks once again to see if it is no longer being powered, and if that is true, it sets the output strength of the node to 15 and updates all nodes that may be affected by this change.

Different node types operates differently:

### Repeater

When a Repeater is updated, the first thing that is checked is if the Repeater should be locked. If that value is different from the current state, the locking state of the Repeater is changed. Since this state change happens during the update, Repeater locking is instant. Then, if the Repeater is not locked and there is not already a tick pending at its node, then whether or not it should be powered is calculated. If this value is different from the current state, a tick is scheduled with the delay of the specific Repeater. The priority of the tick depends on if the output of the Repeater is directly facing another Repeater or Comparator that does not face back toward it. If it is, the priority is `Highest`. If not, but the Repeater is depowering, the priority is `Higher`. Otherwise, the priority is `High`.

When a Repeater is ticked, the first thing that is checked is if the Repeater is locked. If not, then the Repeater checked if it should be powered. If the Repeater is not powered, its state is set to powered. If the Repeater is powered but should not be powered, its state is set to unpowered. Note that a Repeater will become powered here regardless of the input it is receiving, but the same is not true for depowering. If its state is changed, all nodes that may be affected by this change are updated. If the Repeater was just set to powered even though it is not receiving a non-zero input, a tick is scheduled with `Higher` priority with the delay of the Repeater. This is the only node type where a tick is scheduled in the tick function itself.

### Comparator

When a comparator is updated and there is not already a tick pending at its node, it first checks if the comparator has a far override (reading a container through a block). If it does, and regular input to the comparator is not 15, the calculated input to the Comparator is set to the value of the far override. Next, the output strength of the comparator is calculated. If the Comparator is in compare mode and the input strength is greater than or equal to the side input strength, then its output is the input strength, otherwise it is 0. If the Comparator is in subtract mode, then its output strength is its input strength subtracted by its side input strength. This obviously cannot be negative, so it is clamped at 0. Now that the output strength has been calculated, the Comparator checks if that value is different from the current state. If it is, then a tick is scheduled with a delay of 1. The priority of the tick depends on if the output of the Comparator is directly facing another Repeater or Comparator that does not face back toward it. If it is, the priority is `High`, but if not, the priority is `Normal`.

When a Comparator is ticked, the same far override check is performed, and the new output strength is calculated. If that value is different from the current state, the state of the Comparator is changed and any nodes that may be affected by this change is updated.

### Torch

When a Torch is updated and there is not already a tick pending at its node, it checks if the Torch should be off. If that value is different from the current state, then a tick is scheduled with delay 1 and priority `Normal`.

When a Torch is ticked, it checks if the Torch should be off. If that value is different from the current state, the state of the Torch is changed and any nodes that may be affected by this change is updated.

Unlike in vanilla, Torches never burn out.

### Lamp

When a Lamp is updated, it checks if the Lamp should be lit. If a Lamp should be lit but currently is not, then the Lamp state is changed (this is instant). If the Lamp should *not* be lit, but currently is, and no tick is scheduled at its node for a later tick, then a tick is scheduled with delay 2 and priority `Normal`.

When a Lamp is ticked, it checks if the Lamp should be lit. If it should not be bit, but currently is, then the state of the Lamp is changed.

Unlike in vanilla, a Lamp is only updated by the nodes it reads, not by every change near it.

### Trapdoor

When a trapdoor is updated, it checks if it should be powered. If that value is different from its current state, its state is changed (this is instant).

A tick is never scheduled at a Trapdoor node, therefore a Trapdoor is never ticked.

### Wire

If a wire is updated (in an optimized circuit, only wires that are ports are nodes), its signal strength is calculated. If that value is different from its current state, its state is changed (this is instant). Since Wires are leaf nodes, there is no need to update any nodes here since no nodes can be affected by this change.

A tick is never scheduled at a Wire node, therefore a Wire is never ticked.

## Button

When a button is pressed and it is not powered, its state is changed to powered and any nodes that may be affected by this change is updated. Unless a tick is already scheduled at its node, a tick is scheduled after its press duration (10 ticks for stone buttons, 15 for wooden ones) with priority `Normal`.

When a Button is ticked and it is currently powered, its state is changed to unpowered and any nodes that may be affected by this change is updated.

Buttons can never be updated by other nodes.

### Lever

When a lever is flicked, its state is changed to the opposite of its previous state, and any nodes that may be affected by this change is updated (this is instant).

Levers can never be updated nor ticked.

## Tick Ordering

A Redpiler tick is one redstone tick, which is two game ticks. The ticks due in a tick are fixed when it starts; a tick scheduled while ticking runs in a later tick.
Due ticks run by priority, then in the order they were scheduled, as in vanilla. A ticked node updates the nodes it links to right away, so a node ticked later in the same tick reads the result. Except for Lamps, a node has at most one pending tick.

The order in which one change reaches the nodes it links to is not specified.
Vanilla derives it from block positions: a fixed neighbor update order, each component's update pattern, and the iteration order of a Java hash set for redstone wire. Rotating or moving a build can therefore change its behavior.
Redpiler uses the order of each node's outgoing links instead, which approximates vanilla's. The other passes keep this order, but the `Coalesce` pass can move a tick that ran between two merged nodes after both of them.

## The Direct Backend

While the [Direct backend](../crates/redpiler_direct) does not have a JIT compiler, it does implement several optimizations when compared to vanilla:

- Small buffer optimization (SBO) - This not only reduces allocations, but it also helps with memory fragmentation that can lead to cache misses.
- Node sizes are kept as small as possible in memory to allow the node list to fit into small CPU caches.
- Bounds are checked beforehand to avoid performance loss at runtime.
- The tick scheduler is powered by a rotating queue of queues that take into account that there are only 4 possible tick priorities.

# rilc

`rilc` is the command line of the Redpiler crates. It compiles, runs and compares circuits, and runs the [RIL](RIL.md) tests. It reads three kinds of files and never writes to them:

- **Worlds:** a Sponge schematic (`.schem` or `.schematic`), moved so that its corner is at the origin, or a plot file as the server saves it, with its pending ticks and positions counted from the plot's corner. A plot saved by an older version is upgraded in memory only.
- **Circuits:** RIL files (`.ril`). A file that holds several circuits needs the one to use named.
- **Scenarios:** TOML files saying what to do to a world or a circuit, [below](#scenarios).

A setup is a backend and its flags as `/redpiler compile` takes them, like `-b "direct -o -r"`, and can add `--passes=dedup-links,coalesce` to run the named passes instead of the backend's. A RIL circuit has no blocks, so on one the flags that pick wires by their shape (`-d`, `-l` and `-c`) are an error.

`rilc` exits with 0 when the command's claim holds, 1 when it doesn't (a test fails, two setups differ), and 2 when the command can't run, like for a mistake in a file.

## compile

`rilc compile <INPUT> [-b SETUP] [--circuit NAME] [--print-after-all] [-o FILE]` writes the compiled circuit as RIL to stdout, or to `FILE`.
The output starts with comments naming the setup and the input, and then one line per stage: extraction, or `parsed` for a RIL circuit, and each pass, with its nodes, links, their change and its time.
RIL can't hold pending ticks, so a comment says how many the output leaves out.
`--print-after-all` writes the circuit after every stage, each named after its stage, like `@extracted` or `@dedup_links`, so that any stage can be compiled again with `--circuit`.

## Scenarios

```toml
input = "alu.schem"
settle = 1000

[[step]]
toggle = [0, 1, 8]
run = 100

[[step]]
set = [4, 1, 2]
power = 15
run = 20
```

- `input` is the world or RIL file, relative to the scenario. `circuit = "<name>"` picks the circuit of a RIL file that holds several.
- `settle` is optional. It updates every block of a world and runs it until no tick is pending, and fails if that takes more ticks than it gives. The setups then compile the settled world.
- Each `[[step]]` takes at most one action and then runs `run` ticks. `toggle` flips a lever, `press` presses a button, and `set` with `power` sets a pressure plate or another input to a signal strength.
- An input is `[x, y, z]` in a world and a name like `"reset"` in a RIL circuit. An action that the node can't take is an error before anything runs.

## run

`rilc run <SCENARIO> -b SETUP [--repeat N] [--save-outputs FILE]` compiles the input once and plays the scenario on a fresh engine for each repeat.
Like the server, it applies the circuit's changes to the world every 50 ms of wall time.
It reports the compile time, and for each repeat the ticks per second and the time spent applying changes, with their median, minimum and maximum over the repeats.
`--save-outputs` writes the final state of every output port, one per line, sorted by position (by name for a RIL circuit), like `(468, 334, 65) power 11`, with `locked` after a locked repeater. Two runs end the same when their files do, whatever their setups, which a plain text diff shows.

## diff

`rilc diff <SCENARIO> -b A -b B` plays the scenario on two setups, like `-b direct -b "direct -o"`, and compares them before the first tick and after every tick: a world's nodes by position, a RIL circuit's by name.
The setups agree when every port has the same state and the note blocks play equally often.
At the first difference, `diff` reports the tick, the differing ports with their states and the inputs feeding them under each setup, the last action, and the earliest tick and location at which any node both setups have differed.
Every report lists what it didn't compare: the nodes only one setup has, pending ticks, and changes that revert within a tick.

## test

`rilc test <PATH> [--update] [--print-after-all]` runs the tests of a RIL file, or of every RIL file under a directory. `--update` replaces the expected circuit of each failing test with its result, and `--print-after-all` prints each test's circuit after every stage.
