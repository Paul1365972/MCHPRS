# fpga

The `fpga` backend runs the [graph](../Circuits.md) of a plot as a digital circuit.
It writes the graph as SystemVerilog with a module per component and simulates the board that runs it in Verilator.

The backend is behind the `fpga` feature, which is off by default: `cargo build --release --features fpga`.

[Semantics](../Semantics.md#compiled-plots) lists how every compiled plot differs from vanilla. This page covers what is specific to `fpga`.

## Tick Order

The same as [`phase`](Phase.md#tick-order).
After every redstone tick the circuit has the state that `phase` has.

A redstone tick takes a clock cycle per tick priority.
An input takes clock cycles of its own between two redstone ticks.
An input sent while running lands at the next tick boundary the board reaches.
An input while paused runs no tick.
The board runs ticks without being stepped by the host.
It can run periodically, without a rate limit, or to a tick count.
A counted run leaves the board paused.
A periodic tick that falls due while another tick runs or ticks are held starts late.
A rate that asks for less time per tick than a tick takes is reported as behind.
A read returns the tick counter and the state from the same tick boundary.
The board holds ticks while it answers a read or an input.

## Passes

The same as [`phase`](Phase.md#passes).

## Flags

| Flag | Short | Description |
| --- | --- | --- |
| `--optimize` | `-o` | Run the passes. Removing components can reduce the circuit size. |
| `--wire-dot-out` | `-d` | Treat redstone wire without connections (dots) as outputs. |
| `--wire-cross-out` | `-c` | Treat redstone wire connected on all four sides (crosses) as outputs. |
| `--illegal-states-out` | `-l` | Treat redstone wire connected on exactly one side as outputs. |

## Limits

- `fpga` cannot halt on [breakpoints](../Redpiler.md#breakpoints).
- `/redpiler reset` cancels a compile.
- A settle runs at full speed until a read finds no pending tick. It can run past the tick at which the circuit settles.
- The board is read at intervals, so it does not report a component that changes and is back as it was by the next read. A player does not hear an iron trapdoor that opens and closes between two reads.

## Setup

Verilator has to be on the `PATH`.
