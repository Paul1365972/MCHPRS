# fpga

The `fpga` backend runs the [graph](../Circuits.md) of a plot as a digital circuit on a Digilent Basys 3 board, or in Verilator.
It translates the graph to SystemVerilog, builds and programs a bitstream, and controls the board over its serial port.

[Semantics](../Semantics.md#compiled-plots) lists how every compiled plot differs from vanilla. This page covers what is specific to `fpga`.

## Setup

The backend is behind the `fpga` feature, which is off by default: `cargo build --release --features fpga`.
With `--simulate` it only needs Verilator, `make` and a C++ compiler on the `PATH`.
On a board it runs these tools from the `PATH`:

| Tool | For |
| --- | --- |
| Yosys | Synthesis |
| `nextpnr-himbaechel` of [openXC7](https://github.com/openXC7) | Place and route |
| `fasm2frames` and `xc7frames2bit` of prjxray | The bitstream |
| openFPGALoader | Programming the board |

`PRJXRAY_DB_DIR` names the directory of the prjxray database.
`NEXTPNR_XILINX_CHIPDB_DIR` names the directory of nextpnr's chip database `chipdb-xc7a50t.bin`, if nextpnr was installed without it.
The Basys 3's XC7A35T is a die of the XC7A50T.
A small plot compiles for the board in about a minute, a CPU of a few thousand components in about ten.

## Tick Order

The same as [`phase`](Phase.md#tick-order): after every redstone tick the circuit has the state that `phase` has.

## Running

A redstone tick takes one clock cycle per tick priority, at the fastest clock the routed circuit allows.
The board runs its ticks by itself: at a rate, without a limit, or for a number of ticks.
At a rate, a tick that falls due while the board is busy starts late, and the board does not catch up.
Inputs and reads take effect between two ticks.

## Passes

The same as [`phase`](Phase.md#passes).

## Flags

| Flag | Short | Description |
| --- | --- | --- |
| `--optimize` | `-o` | Run the passes. Redstone wire is removed unless it is an output or a probe reads it, which makes the circuit smaller. |
| `--wire-dot-out` | `-d` | Treat redstone wire without connections (dots) as outputs. |
| `--wire-cross-out` | `-c` | Treat redstone wire connected on all four sides (crosses) as outputs. |
| `--illegal-states-out` | `-l` | Treat redstone wire connected on exactly one side as outputs. |
| `--port=<path>` | | The serial port of the board, if more than one is attached. |
| `--simulate` | | Run the circuit in Verilator instead of on a board. |

## Limits

- `fpga` cannot halt on [breakpoints](../Redpiler.md#breakpoints).
- The board is read at intervals, so a component that changes and changes back between two reads is not shown. A player does not hear an iron trapdoor that opens and closes between two reads.
- A settle can run past the tick at which the circuit settles.
- With `--simulate`, the simulation clocks the circuit only while it answers a request and for a short while after, so a run without a tick count advances only as requests arrive and is slow.

## Tests

[`rilc fuzz agreement`](../rilc.md#fuzzing) checks that `fpga` shows what `phase` shows; it needs a rilc built with `--features fpga`.
The circuits in [`crates/redpiler_fpga/tests/circuits`](../../crates/redpiler_fpga/tests/circuits) hold cases that random circuits rarely produce.

```sh
rilc fuzz agreement -b phase -b "fpga --simulate" --cases 100
rilc fuzz agreement crates/redpiler_fpga/tests/circuits/*.ril -b phase -b "fpga --simulate" --cases 100
```

On a board, leave out `--simulate` and add `--jobs 1`.
