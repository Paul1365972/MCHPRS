# Circuits

A circuit is the graph that every [backend](Redpiler.md#backends) but `vanilla` runs.
Its nodes are redstone components and its links carry signal strength between them.
Extraction and the passes work on a `mchprs_redpiler::graph::Graph`. `Circuit::new` checks it and turns it into the `mchprs_redpiler::circuit::Circuit` that a backend builds from.
The behaviour of each node kind is defined in `mchprs_redpiler::behavior`.

[Semantics](Semantics.md) relates this behaviour to vanilla. [RIL](RIL.md) is the text format of a circuit. [Writing a Backend](Writing_a_backend.md) lists the crates and the traits of a backend.

## Nodes

Redpiler turns these blocks into nodes:

| Block | Node kind |
| --- | --- |
| Repeater | Repeater |
| Comparator | Comparator |
| Redstone torch, standing or on a wall | Torch |
| Redstone wire | Wire. `--optimize` leaves out wire that is neither a port nor observed. |
| Button | Button |
| Lever | Lever |
| Wooden, stone or polished blackstone pressure plate | Pressure plate |
| Redstone lamp | Lamp |
| Iron trapdoor | Trapdoor |
| Note block with air above it | Note block |
| Note block with a block above it | Trapdoor, as it plays no note |
| Redstone block | Constant of strength 15 |
| Target block with a strength above 0 | Constant of that strength |
| Powered tripwire hook | Constant of strength 15 |
| Barrel, furnace, hopper, cauldron, composter, cake, end portal frame | Constant of the strength a comparator reads from the block |

A node has the following parts:

- A kind.
- A state: a signal strength from 0 to 15, for a repeater or constant whether it is locked, and for a pressure plate whether an entity touches it.
- At most one pending tick, with a delay in redstone ticks, a priority and an order among the pending ticks.
- A port role: input, output or none.
- Whether it is observed.
- The blocks it stands for.
- Its names, which [RIL](RIL.md) writes as labels.

## Links

A link runs from a source node to a target node.
It has a weight from 0 to 14 and is a default link or a side link.
A link carries the strength of its source minus its weight, or 0 if that is negative.
The weight is the strength the signal loses in the redstone wire between the two components.

The default input of a node is the highest strength its default links carry.
The side input is the highest strength its side links carry.
Only repeaters and comparators have side links.
An input is on when it is above 0.

A node has at most one link of each kind into another node, since of two only the one with the lower weight could decide the input.
`Graph::add_link` is the only way to add a link. Where the graph already holds a link with the same source, target and kind, that link keeps its place among the links of its source and takes the lower of the two weights.

## Ports and Observed Nodes

An input is a node that is driven from outside the circuit. It has no incoming links.
An output is a node that the user reads.

| Role | Nodes |
| --- | --- |
| Input | Levers, buttons, pressure plates |
| Output | Lamps, trapdoors, note blocks, and the wires selected by `--wire-dot-out`, `--wire-cross-out` and `--illegal-states-out` |

`Source::declared` assigns a role to further positions, in place of the role of the table. A declared position that holds no component is refused.
A repeater, torch or comparator declared as input never ticks.

An observed node reports its state changes, and the host writes them into the world.
Without `--io-only`, every node is observed. With it, only ports and the positions in `Source::observed` are.
The server puts the positions of the plot's probes into `Source::observed`.

## Node Behaviour

A node is updated when the strength carried by one of its incoming links changes.
An update either changes the node immediately or schedules a tick.
A tick runs after its delay and can change the node.
When the strength of a node changes, the node updates the targets of its links that now carry another strength.

| Kind | On update | On tick | Delay | Priority |
| --- | --- | --- | --- | --- |
| Repeater | Becomes locked if the side input is on, unlocked otherwise. If it is unlocked, has no pending tick and its output differs from its default input, it schedules a tick. | If it is unlocked: turns off if it is on and the input is off. Turns on if it is off. If it turned on while the input is off, it schedules another tick with priority `Higher`. | Its delay, 1 to 4 | `Highest` if prioritized. Otherwise `High` to turn on and `Higher` to turn off. |
| Torch | If it has no pending tick and is lit while the input is on, or unlit while the input is off, it schedules a tick. | Becomes lit if the input is off, unlit otherwise. | 1 | `Normal` |
| Comparator | If it has no pending tick and its output differs from the output its inputs give, it schedules a tick. | Takes the output its inputs give. | 1 | `High` if prioritized, otherwise `Normal` |
| Lamp | If it is unlit and the input is on, it lights immediately. If it is lit, the input is off and no tick is pending for a later redstone tick, it schedules a tick. | Turns off if the input is off. | 2 | `Normal` |
| Trapdoor | Opens if the input is on, closes otherwise. | | | |
| Wire | Takes the strength of its input. | | | |
| Note block | Becomes powered if the input is on, unpowered otherwise. Plays its note when it becomes powered. | | | |
| Button | | Turns off. | 10 for stone and polished blackstone, 15 for wood | `Normal` |
| Pressure plate | | If it is on and still touched, it schedules another tick. Otherwise it turns off. | 10 | `Normal` |
| Lever, Constant | | | | |

The output of a comparator is computed from its rear input and its side input:

- The rear input is the default input. If the comparator reads a container through a block and the default input is below 15, the rear input is the strength of that container.
- In compare mode, the output is the rear input if it is at least the side input, and 0 otherwise.
- In subtract mode, the output is the rear input minus the side input, or 0 if that is negative.

A repeater or comparator is prioritized when its output faces a repeater or comparator that does not face back at it.

A constant never changes.

### Inputs

Inputs are driven between redstone ticks with `Input::Toggle`, `Input::Press`, `Input::Set(strength)` or `Input::Touch(touched)`.

| Kind | Accepts | Effect |
| --- | --- | --- |
| Lever | `Toggle`, `Set(0)`, `Set(15)` | Turns on or off. |
| Button | `Press` | Turns on and schedules its tick, unless one is pending. A pressed button stays pressed. |
| Pressure plate | `Toggle`, `Touch(false)`, `Touch(true)` | Sets whether an entity touches the plate; `Toggle` flips it. A touched plate that is off turns on and schedules its tick, unless one is pending. A plate that is no longer touched turns off at its next tick. |
| Repeater, torch declared as input | `Toggle`, `Set(0)`, `Set(15)` | Turns on or off. |
| Comparator declared as input | `Set(strength)` | Takes the strength. |

## Tick Order

A tick is one redstone tick, which is two game ticks.

The ticks due in a redstone tick run by priority: `Highest`, `Higher`, `High`, `Normal`. These are `EXTREMELY_HIGH`, `VERY_HIGH`, `HIGH` and `NORMAL` in vanilla.
A tick scheduled during a redstone tick runs in a later redstone tick.

How the ticks of one priority run, and when a change updates what it powers, is the backend's choice. [Semantics](Semantics.md#backends) compares the backends.

## Passes

A pass rewrites the graph before the backend builds it.
Passes are the types in the modules of `mchprs_redpiler::passes` that implement `Pass`, and the [`coalesce` of `phase`](backends/Phase.md#coalesce) in `mchprs_redpiler_phase::coalesce`.
A backend chooses which passes it runs and in which order; its page lists them.
A pass must not change the result of a build that is not locational.
It may change the result of a locational build, the same way a recompile can, but only to a result that the backend's tick order allows for the original graph.

| Pass | Effect |
| --- | --- |
| `fold` | Turns every node whose state the analysis below proves fixed into a constant and removes its incoming links. Inputs and nodes with a pending tick are not folded. Also removes every link whose weight is at least the highest strength its source can reach. |
| `unify-constants` | Moves the links of all constants to a single constant of strength 15 and raises each weight by the strength its constant lacked. A link that carried no strength is removed. Constants that are neither ports nor observed are removed. |
| `prune-orphans` | Removes every node that is not observed, has no pending tick and does not lead to such a node through links. |
| `sort-links` | Sorts the links of every node by the kind of their target, then by weight. This changes only the order in which a node updates what it powers. |
| `coalesce` | Merges nodes that agree in kind, port role, state, incoming links and whether they are observed into one node that stands for the blocks of all of them. The weight of a link that carries only on or off into a node that reads only on or off is not compared. Inputs, nodes with a pending tick and nodes that are their own input are not merged. Two links into the same input of a target become one. Nodes are merged only if the merged node [keeps the order](backends/Direct.md#coalesce) in which they schedule ticks, which the tick order of `direct` needs. |
| `coalesce` of `phase` | Merges nodes as [`phase`](backends/Phase.md#coalesce) describes, also in loops and with pending ticks. Only `phase` and `fpga` run it. |

`fold`, the `coalesce` of `phase` and `fpga` use the analysis in `mchprs_redpiler::analysis::ss_sets`, which computes for every node a set that contains every strength the node can reach.

## Invariants

`Circuit::new` and the RIL parser reject a graph that breaks one of these rules:

- Every link has a weight below 15.
- The source of every link is a kind that emits a signal: not a lamp, trapdoor, wire or note block.
- The target of every link is a kind that reads a signal and is not an input.
- Side links end at repeaters and comparators.
- Every input is a kind that can be driven.
- Only repeaters and constants are locked, and only pressure plates are touched.
- A repeater has a delay from 1 to 4, a button stays pressed for 10 or 15 ticks, and a note block has a note from 0 to 24.
- A node that is on or off has strength 0 or 15.
- A pending tick belongs to a node that ticks, has a priority that the kind of the node schedules, and a delay from 1 to the longest that the kind schedules with that priority.
- No node is [stale](Semantics.md#baseline-requirements).

The RIL parser also refuses two links of one kind from one node into another, which a graph [cannot hold](#links).
