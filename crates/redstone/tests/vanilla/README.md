# Traces recorded from Minecraft 1.20.4

Each `.trace` is a world of supported blocks as the real 1.20.4 server ran it, recorded once with an external harness that is not part of this repository.
The `gen-*` worlds are random, picked from a larger recording so that together they show every block kind, every property value and every pair of touching kinds that the recording did; the `fuzz-*-min` ones are minimized worlds on which an earlier interpreter differed; the `plate-*` ones are written by hand for the timing of pressure plates.
`tests/vanilla.rs` replays every trace on the interpreter and compares each observed block after every redstone tick (two game ticks) and after every group of actions.
The worlds are structurally stable, start settled with no pending ticks, and are driven only by uses of levers and buttons and by steps onto and off pressure plates.
`origin` and `size` give the observed box, `state` numbers a vanilla block state, and `start`, `wait <game ticks>`, `use <positions>`, `stand_on <positions>` and `step_off <positions>` list the blocks that changed as `x,y,z=state`, relative to the origin.
A `wait` changes its blocks in its last tick, and nothing changes in the ticks before it.
A `use` or `step_off` happens between two redstone ticks, after an even game tick.
A `stand_on` takes one redstone tick: the entity appears between its two game ticks, so vanilla presses the plate in its second game tick's entity phase, after that tick's scheduled ticks, and the replay calls `Interpreter::stand_on` after the tick.
The entity is an armor stand with gravity at the bottom center of the plate's block, which presses wooden and stone plates alike and stays put until `step_off` removes it; a `stand_on` right after a `step_off` of the same plate is another armor stand.
Compared are all block states and the comparators' `OutputSignal`; only a hopper's `enabled` is ignored, since hoppers never lock here, and sounds are not recorded.
No trace relies on torch burnout or observers, which the interpreter leaves out on purpose.
