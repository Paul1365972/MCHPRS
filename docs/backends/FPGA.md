# fpga

The `fpga` backend runs the [graph](../Circuits.md) of a plot as a digital circuit on an FPGA board.
It writes the graph as SystemVerilog with a module per component, builds and loads a bitstream, and controls the board over its serial port.

It supports the Digilent Basys 3 with the open toolchain [openXC7](https://github.com/openXC7).
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
The clock uses the fastest supported PLL setting that the timing result allows.
`--clock` overrides that limit.

## Passes

The same as [`phase`](Phase.md#passes).

## Mapping

The backend finds the signal strengths each node can produce.
It omits comparisons whose result is fixed by those strengths.
The analysis includes the initial state and every possible input.

## Flags

| Flag | Short | Description |
| --- | --- | --- |
| `--optimize` | `-o` | Run the passes. Removing components can reduce the circuit size. |
| `--io-only` | `-i` | Only inputs, outputs and probed components show their state. The board cannot report the state of the others. |
| `--wire-dot-out` | `-d` | Treat redstone wire without connections (dots) as outputs. |
| `--wire-cross-out` | `-c` | Treat redstone wire connected on all four sides (crosses) as outputs. |
| `--illegal-states-out` | `-l` | Treat redstone wire connected on exactly one side as outputs. |
| `--port=<path>` | | The serial port of the board. Only needed if more than one board is attached. |
| `--clock=<MHz>` | | The highest PLL frequency to use, overriding the timing result. Above that result the circuit can compute wrong results. |
| `--simulate` | | Run the circuit in Verilator instead of on a board. |

## Limits

- `fpga` cannot halt on [breakpoints](../Redpiler.md#breakpoints).
- `/redpiler reset` cancels a compile.
- A mode change, an input and a read of the observed state each take an exchange over the serial port.
- A settle runs at full speed until a read finds no pending tick. It can run past the tick at which the circuit settles.
- The board is read at intervals, so it does not report a component that changes and is back as it was by the next read. A player does not hear an iron trapdoor that opens and closes between two reads.
- With `--io-only`, `/redpiler reset` restores the plot to its state before compiling.
- With `--io-only`, the board does not report pending ticks, so `/radvance settle` is unavailable.

## Setup

`fpga` needs Linux. On Windows, run the server in WSL 2 and attach the board with [usbipd-win](https://github.com/dorssel/usbipd-win).

Yosys, nextpnr, prjxray and openFPGALoader have to be on the `PATH`, and Verilator for `--simulate`.
The flake in [`crates/redpiler_fpga/toolchain`](../../crates/redpiler_fpga/toolchain/flake.nix) pins all of them:

```sh
nix develop path:crates/redpiler_fpga/toolchain
```

The user needs access to the serial port and the USB device of the board (USB id `0403:6010`).
The USB chip buffers answers until its latency timer expires.
This udev rule sets the timer whenever the board is attached:

```
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```
