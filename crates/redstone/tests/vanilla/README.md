# Traces recorded from Minecraft 1.20.4

Each `.trace` is a world of supported blocks as the real 1.20.4 server ran it, recorded once with an external harness that is not part of this repository.
`tests/vanilla.rs` replays every trace on the interpreter and compares each observed block after every redstone tick (two game ticks) and after every group of actions.
A `wait` changes its blocks in its last tick, and nothing changes in the ticks before it.
A `use` or `step_off` happens between two redstone ticks, after an even game tick.
Compared are all block states and the comparators' `OutputSignal`; only a hopper's `enabled` is ignored, since hoppers never lock here, and sounds are not recorded.
No trace relies on torch burnout or observers, which the interpreter leaves out on purpose.
