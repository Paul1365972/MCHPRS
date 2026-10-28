// Starts the redstone ticks, counts them and hands inputs from the host to the nodes.
//
// A redstone tick takes one clock per priority. In each of those clocks one firing bit is high,
// and the nodes whose tick has that priority fire.
//
// What every node module builds on:
// - A node fires from its inputs of the clock before, which hold their state at the start of its
//   priority. Its output is valid in the same clock, and every node takes it at the end of that
//   clock.
// - Every node updates on every clock. An update from an unchanged input changes nothing, since no
//   node starts stale.
// - A strobe of the circuit's ports only loads registers, so no path leads from the handshake
//   into a node. A drive takes three clocks, each ending in registers: the sequencer registers it,
//   the addressed node registers that it is meant, and that node then changes like a firing node.
// - At least one clock passes before the first tick or drive, so a register that holds an input
//   of the clock before and is read only as its node fires needs no initial value.
module sequencer #(
    parameter int COPIES
) (
    input logic clock,
    input logic hold,
    input logic run_start,
    input pkg::mode_t run_mode,
    input logic [pkg::COUNT_BITS-1:0] run_count,
    input logic drive_valid,
    input logic [23:0] drive_node,
    input logic [5:0] drive_input,
    output logic idle,
    output logic [63:0] ticks,
    output logic paused,
    output logic [COPIES*pkg::PRIORITIES-1:0] firing,
    output logic taken_valid = 0,
    output logic [23:0] taken_node = 0,
    output logic [5:0] taken_input = 0
);
    // One-hot: bit p is high in the clock in which priority p fires, and all bits are low while
    // no tick runs.
    logic [pkg::PRIORITIES-1:0] phase = 0;
    logic last_phase;
    // A tick ends at most once every `pkg::PRIORITIES` clocks, and the count is read only while
    // ticks are held, so the high half may add the carry of the low half a clock later.
    logic [31:0] ticks_low = 0;
    logic [31:0] ticks_high = 0;
    logic low_wrapped = 0;
    pkg::mode_t mode = pkg::PAUSED;
    // Counted runs: the ticks left to begin. Periodic runs: the clocks until the next tick is due.
    logic [pkg::COUNT_BITS-1:0] left = 0;
    logic [pkg::COUNT_BITS-1:0] period = 0;
    // Whether the mode allows a tick to begin. A register, which keeps the mode and the counter off
    // the path of a tick.
    logic ready = 0;
    // A tick begins in the next clock. Decided a clock ahead, so that `firing` is a plain shift
    // register in every copy.
    logic beginning;
    logic applying = 0;

    assign last_phase = phase[pkg::PRIORITIES-1];
    assign ticks = {ticks_high, ticks_low};
    assign idle = phase == 0 && !taken_valid && !applying;
    assign paused = mode == pkg::PAUSED;
    assign beginning = !hold && ready && (phase == 0 || last_phase);

    always_ff @(posedge clock) begin
        phase <= {phase[pkg::PRIORITIES-2:0], beginning};

        if (last_phase) ticks_low <= ticks_low + 1'b1;
        low_wrapped <= last_phase && &ticks_low;
        if (low_wrapped) ticks_high <= ticks_high + 1'b1;

        unique case (mode)
            pkg::PAUSED: ready <= 0;
            pkg::UNLIMITED: ready <= 1;
            pkg::COUNTED: begin
                ready <= left != 0;
                if (beginning) left <= left - 1'b1;
                if (left == 0) mode <= pkg::PAUSED;
            end
            pkg::PERIODIC: begin
                if (left == 1) begin
                    ready <= 1;
                    left <= period;
                end else begin
                    if (beginning) ready <= 0;
                    left <= left - 1'b1;
                end
            end
        endcase
        if (run_start) begin
            mode <= run_mode;
            left <= run_count;
            period <= run_count;
            ready <= 0;
        end

        taken_valid <= drive_valid;
        if (drive_valid) begin
            taken_node <= drive_node;
            taken_input <= drive_input;
        end
        applying <= taken_valid;
    end

    for (genvar copy = 0; copy < COPIES; copy++) begin : copies
        firing_copy firing_copy (
            .clock(clock), .beginning(beginning),
            .firing(firing[copy*pkg::PRIORITIES +: pkg::PRIORITIES])
        );
    end
endmodule
